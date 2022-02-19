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

void fill_config(ros::NodeHandle& nh);

int main(int argc, char** argv) {
    ros::init(argc, argv, "limovelo");
    ros::NodeHandle nh;

    // Fill configurations Params with YAML
    fill_config(nh);

    // Modules
        Publishers publish(nh);
        Accumulator& accum = Accumulator::getInstance();
        Compensator comp = Compensator ();
        Mapper& map = Mapper::getInstance();
        Localizator& loc = Localizator::getInstance();

    // Subscribers
        // LiDAR
        ros::Subscriber lidar_sub;
        if (Config.LiDAR_type != LIDAR_TYPE::Livox) {
            lidar_sub = nh.subscribe(
                Config.points_topic, 1000,
                &Accumulator::receive_lidar, &accum
            );
        } else {
            lidar_sub = nh.subscribe(
                Config.points_topic, 1000,
                &Accumulator::receive_livox, &accum
            );
        }

        // IMU
        ros::Subscriber imu_sub = nh.subscribe(
            Config.imus_topic, 1000,
            &Accumulator::receive_imu, &accum
        );

    // Time variables
    double t1, t2;
    t2 = DBL_MAX;

    // Delta (= t2 - t1): size of the field of view we use to localize
    double delta = Config.Heuristic.deltas.front();
    
    ros::Rate rate(5000);

    while (ros::ok()) {
        
        // The accumulator received enough data to start
        while (accum.ready()) {
            
            // Step 0. TIME MANAGEMENT
            // Define time interval [t1, t2] which we will use to localize ourselves
                
                // Real-time, define t2 as the latest time
                if (Config.real_time) t2 = accum.latest_time();
                // Not real time, define t2 as prev_t2 + delta, but don't go into the future
                else t2 = std::min(t2 + delta, accum.latest_time());
                
                // Update delta value
                delta = accum.update_delta(Config.Heuristic, t2);

                // Define t1 but don't use to localize repeated points
                t1 = std::max(t2 - delta, loc.last_time_updated);
                // Check if interval has enough field of view
                if (t2 - t1 < delta - 1e-6) break;

            // Step 1. LOCALIZATION

                // Integrate IMUs up to t2
                loc.propagate_to(t2);

                // Compensated pointcloud given a path
                Points compensated = comp.compensate(t1, t2);
                Points ds_compensated = comp.downsample(compensated);
                if (ds_compensated.size() < Config.MAX_POINTS2MATCH) break; 

                // Localize points in map
                loc.localize(ds_compensated, t2);
                State Xt2 = loc.latest_state();
                accum.add(Xt2, t2);
                publish.state(Xt2, false);
                publish.tf(Xt2);

                // Publish pointcloud used to localize
                Points global_compensated = Xt2 * Xt2.I_Rt_L() * ds_compensated;
                publish.pointcloud(global_compensated, true);

            // Step 2. MAPPING

                // Add updated points to map (mapping online)
                if (Config.mapping_online) {
                    map.add(global_compensated, t2, true);
                    publish.pointcloud(global_compensated, false);
                    
                    if (Config.print_extrinsics) publish.extrinsics(Xt2);
                }
                // Add updated points to map (mapping offline)
                else if (map.hasToMap(t2)) {
                    State Xt2 = loc.latest_state();
                    // Map points at [t2 - FULL_ROTATION_TIME, t2]
                    Points full_compensated = comp.compensate(t2 - Config.full_rotation_time, t2);
                    Points global_full_compensated = Xt2 * Xt2.I_Rt_L() * full_compensated;
                    Points global_full_ds_compensated = comp.downsample(global_full_compensated);

                    if (global_full_ds_compensated.size() < Config.MAX_POINTS2MATCH) break; 
                    if (Config.print_extrinsics) publish.extrinsics(Xt2);

                    map.add(global_full_ds_compensated, t2, true);
                    publish.pointcloud(global_full_ds_compensated, false);
                }

            // Step 3. ERASE OLD DATA

                // Empty too old LiDAR points
                accum.clear_lidar(t2 - Config.empty_lidar_time);

            // Trick to call break in the middle of the program
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void fill_config(ros::NodeHandle& nh) {
    // Read YAML parameters
    nh.param<bool>("mapping_online", Config.mapping_online, true);
    nh.param<bool>("real_time", Config.real_time, true);
    nh.param<bool>("estimate_extrinsics", Config.estimate_extrinsics, false);
    nh.param<bool>("print_extrinsics", Config.print_extrinsics, false);
    nh.param<int>("ds_rate", Config.ds_rate, 4);
    nh.param<int>("MAX_NUM_ITERS", Config.MAX_NUM_ITERS, 3);
    nh.param<std::vector<double>>("LIMITS", Config.LIMITS, std::vector<double> (23, 0.001));
    nh.param<int>("NUM_MATCH_POINTS", Config.NUM_MATCH_POINTS, 5);
    nh.param<int>("MAX_POINTS2MATCH", Config.MAX_POINTS2MATCH, 10);
    nh.param<double>("MAX_DIST_PLANE", Config.MAX_DIST_PLANE, 2.0);
    nh.param<float>("PLANES_THRESHOLD", Config.PLANES_THRESHOLD, 0.1f);
    nh.param<float>("PLANES_CHOOSE_CONSTANT", Config.PLANES_CHOOSE_CONSTANT, 9.0f);
    nh.param<std::string>("LiDAR_type", Config.LiDAR_type, "unknown");
    nh.param<double>("LiDAR_noise", Config.LiDAR_noise, 0.001);
    nh.param<double>("min_dist", Config.min_dist, 3.);
    nh.param<double>("imu_rate", Config.imu_rate, 400);
    nh.param<double>("degeneracy_threshold", Config.degeneracy_threshold, 5.d);
    nh.param<bool>("print_degeneracy_values", Config.print_degeneracy_values, false);
    nh.param<double>("full_rotation_time", Config.full_rotation_time, 0.1);
    nh.param<double>("empty_lidar_time", Config.empty_lidar_time, 20.);
    nh.param<double>("real_time_delay", Config.real_time_delay, 1.);
    nh.param<double>("covariance_gyroscope", Config.cov_gyro, 1e-4);
    nh.param<double>("covariance_acceleration", Config.cov_acc, 1e-2);
    nh.param<double>("covariance_bias_gyroscope", Config.cov_bias_gyro, 1e-5);
    nh.param<double>("covariance_bias_acceleration", Config.cov_bias_acc, 1e-4);
    nh.param<double>("wx_MULTIPLIER", Config.wx_MULTIPLIER, 1);
    nh.param<double>("wy_MULTIPLIER", Config.wy_MULTIPLIER, 1);
    nh.param<double>("wz_MULTIPLIER", Config.wz_MULTIPLIER, 1);
    nh.param<std::string>("points_topic", Config.points_topic, "/velodyne_points");
    nh.param<std::string>("imus_topic", Config.imus_topic, "/vectornav/IMU");
    nh.param<std::vector<double>>("/Heuristic/times", Config.Heuristic.times, {});
    nh.param<std::vector<double>>("/Heuristic/deltas", Config.Heuristic.deltas, {0.1});
    nh.param<std::vector<float>>("initial_gravity", Config.initial_gravity, {0.0, 0.0, -9.807});
    nh.param<std::vector<float>>("I_Translation_L", Config.I_Translation_L, std::vector<float> (3, 0.));
    nh.param<std::vector<float>>("I_Rotation_L", Config.I_Rotation_L, std::vector<float> (9, 0.));
}