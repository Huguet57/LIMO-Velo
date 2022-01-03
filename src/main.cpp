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

int main(int argc, char** argv) {
    ros::init(argc, argv, "limovelo");
    ros::NodeHandle nh;

    // Get YAML parameters
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
    nh.param<double>("LiDAR_noise", Config.LiDAR_noise, 0.001);
    nh.param<double>("min_dist", Config.min_dist, 3.);
    nh.param<double>("imu_rate", Config.imu_rate, 400);
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
    nh.param<std::vector<double>>("/Heuristic/times", Config.Heuristic.times, {1.});
    nh.param<std::vector<double>>("/Heuristic/deltas", Config.Heuristic.deltas, {0.1, 0.01});
    nh.param<std::vector<float>>("initial_gravity", Config.initial_gravity, {0.0, 0.0, -9.807});
    nh.param<std::vector<float>>("I_Translation_L", Config.I_Translation_L, std::vector<float> (3, 0.));
    nh.param<std::vector<float>>("I_Rotation_L", Config.I_Rotation_L, std::vector<float> (9, 0.));

    // Objects
    Publishers publish(nh);
    Accumulator& accum = Accumulator::getInstance();
    Compensator comp = Compensator (publish);
    Mapper& map = Mapper::getInstance();
    Localizator& KF = Localizator::getInstance();

    // Subscribers
    ros::Subscriber lidar_sub = nh.subscribe(
        Config.points_topic, 1000,
        &Accumulator::receive_lidar, &accum
    );

    ros::Subscriber imu_sub = nh.subscribe(
        Config.imus_topic, 1000,
        &Accumulator::receive_imu, &accum
    );

    ros::Subscriber tfs_sub = nh.subscribe(
        "/tf", 1000,
        &Publishers::receive_tf, &publish
    );

    ros::Rate rate(10);

    // If not real time, define an artificial clock
    double clock = -1;

    while (ros::ok()) {
        
        while (accum.ready()) {
            
            double t2;
            if (Config.real_time) {
                // Should be t2 = ros::Time::now() - delay
                double latest_imu_time = accum.BUFFER_I.front().time;
                t2 = latest_imu_time - Config.real_time_delay; 
            } else {
                if (clock < 0) clock = accum.initial_time;
                t2 = clock - Config.real_time_delay; 
                clock += accum.delta;
            }

            // Refine delta if need to be
            rate = accum.refine_delta(t2);
            double t1 = t2 - accum.delta;

            if (Config.mapping_online or (not Config.mapping_online and map.exists())) {
                // Integrate from t1 to t2
                KF.propagate_to(t2);

                // Compensated pointcloud given a path
                Points points = accum.get_points(t1, t2);
                States path_taken = comp.integrate_imus(t1, t2);
                PointCloud compensated = comp.compensate(path_taken, points);
                if (compensated.size() < Config.MAX_POINTS2MATCH) break; 

                // Localize points in map
                KF.update(compensated);
                State Xt2 = KF.latest_state();
                accum.add(Xt2, t2);
                publish.state(Xt2, false);
                publish.tf(Xt2);

                // Publish planes
                Planes matches = map.match(Xt2, compensated);
                publish.planes(Xt2, matches);

                // Publish compensated
                PointCloud global_compensated = Xt2 * Xt2.I_Rt_L() * compensated;
                publish.pointcloud(global_compensated);

                // Map at the same time (online)
                if (Config.mapping_online) {
                    map.add(global_compensated, t2, false);
                    publish.full_pointcloud(global_compensated);
                }
            }
            
            // Add updated points to map (offline)
            if (not Config.mapping_online and map.hasToMap(t2)) {
                PointCloud global_full_compensated = comp.compensate(t2 - Config.full_rotation_time, t2, true);

                if (Config.print_extrinsics) publish.extrinsics(KF.latest_state());

                map.add(global_full_compensated, t2, true);
                publish.full_pointcloud(global_full_compensated);
            }

            // Empty too old LiDAR points
            accum.empty_lidar(t2 - Config.empty_lidar_time);

            // Trick to call break in the middle of the program
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}