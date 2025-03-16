#ifndef __OBJECTS_H__
#define __OBJECTS_H__
#include "Headers/Common.hpp"
#include "Headers/Utils.hpp"
#include "Headers/Objects.hpp"
#include "Headers/Publishers.hpp"
#include "Headers/PointClouds.hpp"
#include "Headers/Accumulator.hpp"
#include "Headers/Compensator.hpp"
#include "Headers/Localizator.hpp"
#include "Headers/Mapper.hpp"
#endif

Params Config;

using namespace std::chrono_literals;

void fill_config(rclcpp::Node::SharedPtr node);

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("limovelo");

    Points buffer_mapped_points_;

    // Fill configurations Params con YAML
    fill_config(node);

    // Objects
    Publishers publish(node);
    Accumulator& accum = Accumulator::getInstance();
    Compensator comp = Compensator ();
    Mapper& map = Mapper::getInstance();
    Localizator& loc = Localizator::getInstance();

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub = 
        node->create_subscription<sensor_msgs::msg::PointCloud2>(
        Config.points_topic, 1, 
        std::bind(&Accumulator::receive_lidar, &accum, std::placeholders::_1)
    );

    rclcpp::Subscription<common_msgs::msg::State>::SharedPtr state_sub = 
        node->create_subscription<common_msgs::msg::State>(
        Config.imus_topic, 1,
        std::bind(&Accumulator::receive_state, &accum, std::placeholders::_1)
    );

    // Time variables
    double t1, t2;
    t2 = DBL_MAX;

    // (Delta = t2 - t1) Size of the field of view we use to localize
    double delta = Config.Initialization.deltas.front();
    
    rclcpp::Rate rate(5000);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (rclcpp::ok()) {
        
        // The accumulator received enough data to start
        while (accum.ready()) {
            
            // Step 0. TIME MANAGEMENT
            // Define time interval [t1, t2] which we will use to localize ourselves
                
                // Real-time, define t2 as the latest time
                if (Config.real_time) t2 = accum.latest_time();
                // Not real time, define t2 as prev_t2 + delta, but don't go into the future
                else t2 = std::min(t2 + delta, accum.latest_time());
                
                // Update delta value
                delta = accum.update_delta(Config.Initialization, t2);

                // Define t1 but don't use to localize repeated points
                t1 = std::max(t2 - delta, loc.last_time_updated);
                // Check if interval has enough field of view
                if (t2 - t1 < delta - 1e-6) {
                    if (Config.print_debug) RCLCPP_WARN(rclcpp::get_logger("limovelo"), "Not enough field of view to localize");
                    break;
                };

            // Step 1. LOCALIZATION

                // Integrate IMUs up to t2
                loc.propagate_to(t2);

                // Compensated pointcloud given a path
                Points compensated = comp.compensate(t1, t2);
                Points ds_compensated = comp.downsample(compensated);
                if (ds_compensated.size() < Config.MAX_POINTS2MATCH) {
                    if (Config.print_debug) RCLCPP_WARN(rclcpp::get_logger("limovelo"), "ds_compensated.size(): %d, Config.MAX_POINTS2MATCH: %d", ds_compensated.size(), Config.MAX_POINTS2MATCH);
                    if (Config.print_debug) RCLCPP_WARN(rclcpp::get_logger("limovelo"), "Not enough points to match");
                    break;
                }

                // Localize points in map
                loc.correct(ds_compensated, t2);
                State Xt2 = loc.latest_state();
                accum.add(Xt2, t2);
                publish.state(Xt2, false);
                publish.tf(Xt2);

                // Publish pointcloud used to localize
                Points global_compensated = Xt2 * Xt2.I_Rt_L() * compensated;
                Points global_ds_compensated = Xt2 * Xt2.I_Rt_L() * ds_compensated;
                // publish.pointcloud(global_ds_compensated, true);

                // Publish updated extrinsics
                if (Config.print_extrinsics) publish.extrinsics(Xt2);

            // Step 2. MAPPING

                // Update map and add points to buffer
                if (Config.mapping) {
                    map.add(global_ds_compensated, t2, true);
                    buffer_mapped_points_.insert(
                        buffer_mapped_points_.end(),   // Position
                        global_compensated.begin(), // First
                        global_compensated.end()    // Last
                    );
                    // Publish historical pointcloud
                    publish.pointcloud(buffer_mapped_points_, false);
                    if (Config.print_debug) RCLCPP_INFO(rclcpp::get_logger("limovelo"), "\033[1;32mHistoric pcl published, size:\033[0m %d", buffer_mapped_points_.size());
                }
                else {
                    map.add(global_ds_compensated, t2, true);
                    if (Config.high_quality_publish) publish.pointcloud(global_compensated, false);                    
                    else publish.pointcloud(global_ds_compensated, false);                    
                }

            // Step 3. ERASE OLD DATA

                // Empty too old LiDAR points from the accumulator
                accum.clear_lidar(t2 - Config.time_without_clear_BUFFER_L);

                // Remove oldest points from buffer_mapped_points_
                while (!buffer_mapped_points_.empty() && buffer_mapped_points_.front().time < t2 - Config.time_without_clear_buffer_mapped_points) {
                    buffer_mapped_points_.pop_front();
                }

            // Trick to call break in the middle of the program
            break;
        }

        executor.spin_once();
        rate.sleep();
    }

    return 0;
}

void fill_config(rclcpp::Node::SharedPtr node) {
    // Read YAML parameters
    node->declare_parameter("mapping", true);
    node->get_parameter("mapping", Config.mapping);

    node->declare_parameter("print_debug", false);
    node->get_parameter("print_debug", Config.print_debug);

    node->declare_parameter("real_time", true);
    node->get_parameter("real_time", Config.real_time);

    node->declare_parameter("estimate_extrinsics", false);
    node->get_parameter("estimate_extrinsics", Config.estimate_extrinsics);
    
    node->declare_parameter("print_extrinsics", false);
    node->get_parameter("print_extrinsics", Config.print_extrinsics);
    
    node->declare_parameter("downsample_rate", 4);
    node->get_parameter("downsample_rate", Config.downsample_rate);
    
    node->declare_parameter("downsample_prec", 0.2);
    node->get_parameter("downsample_prec", Config.downsample_prec);
    
    node->declare_parameter("high_quality_publish", false);
    node->get_parameter("high_quality_publish", Config.high_quality_publish);
    
    node->declare_parameter("MAX_NUM_ITERS", 3);
    node->get_parameter("MAX_NUM_ITERS", Config.MAX_NUM_ITERS);
    
    node->declare_parameter("LIMITS", std::vector<double>{23, 0.001});
    node->get_parameter("LIMITS", Config.LIMITS);
    
    node->declare_parameter("NUM_MATCH_POINTS", 5);
    node->get_parameter("NUM_MATCH_POINTS", Config.NUM_MATCH_POINTS);
    
    node->declare_parameter("MAX_POINTS2MATCH", 10);
    node->get_parameter("MAX_POINTS2MATCH", Config.MAX_POINTS2MATCH);
    
    node->declare_parameter("MAX_DIST_PLANE", 2.0);
    node->get_parameter("MAX_DIST_PLANE", Config.MAX_DIST_PLANE);
    
    node->declare_parameter("PLANES_THRESHOLD", 0.1f);
    node->get_parameter("PLANES_THRESHOLD", Config.PLANES_THRESHOLD);
    
    node->declare_parameter("PLANES_CHOOSE_CONSTANT", 9.0f);
    node->get_parameter("PLANES_CHOOSE_CONSTANT", Config.PLANES_CHOOSE_CONSTANT);
    
    node->declare_parameter("LiDAR_type", "unknown");
    node->get_parameter("LiDAR_type", Config.LiDAR_type);
    
    node->declare_parameter("LiDAR_noise", 0.001);
    node->get_parameter("LiDAR_noise", Config.LiDAR_noise);
    
    node->declare_parameter("min_dist", 3.);
    node->get_parameter("min_dist", Config.min_dist);
    
    node->declare_parameter("imu_rate", 400.0);
    node->get_parameter("imu_rate", Config.imu_rate);
    
    node->declare_parameter("degeneracy_threshold", 5.d);
    node->get_parameter("degeneracy_threshold", Config.degeneracy_threshold);
    
    node->declare_parameter("print_degeneracy_values", false);
    node->get_parameter("print_degeneracy_values", Config.print_degeneracy_values);
    
    node->declare_parameter("full_rotation_time", 0.1);
    node->get_parameter("full_rotation_time", Config.full_rotation_time);
    
    node->declare_parameter("time_without_clear_BUFFER_L", 20.);
    node->get_parameter("time_without_clear_BUFFER_L", Config.time_without_clear_BUFFER_L);

    node->declare_parameter("time_without_clear_buffer_mapped_points", 20.);
    node->get_parameter("time_without_clear_buffer_mapped_points", Config.time_without_clear_buffer_mapped_points);
    
    node->declare_parameter("real_time_delay", 1.);
    node->get_parameter("real_time_delay", Config.real_time_delay);
    
    node->declare_parameter("covariance_gyroscope", 1.0e-4);
    node->get_parameter("covariance_gyroscope", Config.cov_gyro);
    
    node->declare_parameter("covariance_acceleration", 1.0e-2);
    node->get_parameter("covariance_acceleration", Config.cov_acc);
    
    node->declare_parameter("covariance_bias_gyroscope", 1.0e-5);
    node->get_parameter("covariance_bias_gyroscope", Config.cov_bias_gyro);
    
    node->declare_parameter("covariance_bias_acceleration", 1.0e-4);
    node->get_parameter("covariance_bias_acceleration", Config.cov_bias_acc);
    
    node->declare_parameter("wx_MULTIPLIER", 1.);
    node->get_parameter("wx_MULTIPLIER", Config.wx_MULTIPLIER);
    
    node->declare_parameter("wy_MULTIPLIER", 1.);
    node->get_parameter("wy_MULTIPLIER", Config.wy_MULTIPLIER);
    
    node->declare_parameter("wz_MULTIPLIER", 1.);
    node->get_parameter("wz_MULTIPLIER", Config.wz_MULTIPLIER);
    
    node->declare_parameter("points_topic", "/velodyne_points");
    node->get_parameter("points_topic", Config.points_topic);
    
    node->declare_parameter("imus_topic", "/vectornav/IMU");
    node->get_parameter("imus_topic", Config.imus_topic);
    
    node->declare_parameter("offset_beginning", false);
    node->get_parameter("offset_beginning", Config.offset_beginning);
    
    node->declare_parameter("stamp_beginning", false);
    node->get_parameter("stamp_beginning", Config.stamp_beginning);
    
    node->declare_parameter("Initialization.times", std::vector<double>{});
    node->get_parameter("Initialization.times", Config.Initialization.times);
    
    node->declare_parameter("Initialization.deltas", std::vector<double>{Config.full_rotation_time});
    node->get_parameter("Initialization.deltas", Config.Initialization.deltas);
    
    node->declare_parameter("initial_gravity", std::vector<double>{0.0, 0.0, -9.807});
    node->get_parameter("initial_gravity", Config.initial_gravity);
    
    node->declare_parameter("I_Translation_L", std::vector<double>{3, 0.});
    node->get_parameter("I_Translation_L", Config.I_Translation_L);
    
    node->declare_parameter("I_Rotation_L",std::vector<double>{9, 0.});
    node->get_parameter("I_Rotation_L", Config.I_Rotation_L);
}
