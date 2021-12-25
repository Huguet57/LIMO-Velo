#ifndef __OBJECTS_H__
#define __OBJECTS_H__
#include "Common.hpp"
#include "Utils.hpp"
#include "Objects.hpp"
#include "Publishers.hpp"
#include "PointClouds.hpp"
#include "Accumulator.hpp"
#include "Compensator.hpp"
#endif

Params Config;

int main(int argc, char** argv) {
    ros::init(argc, argv, "limovelo");
    ros::NodeHandle nh;

    // Get YAML parameters
    nh.param<double>("delta", Config.delta, 0.025);
    nh.param<int>("rate", Config.rate, (int) 1./Config.delta);
    nh.param<int>("ds_rate", Config.ds_rate, 4);
    nh.param<double>("min_dist", Config.min_dist, 3.);
    nh.param<double>("empty_lidar_time", Config.empty_lidar_time, 20.);
    nh.param<double>("real_time_delay", Config.real_time_delay, 1.);
    nh.param<std::string>("points_topic", Config.points_topic, "/velodyne_points");
    nh.param<std::string>("imus_topic", Config.imus_topic, "/vectornav/IMU");

    Accumulator accum;
    Publishers publish(nh);
    Compensator comp(accum, publish, Config.delta);

    // Subscribers
    ros::Subscriber lidar_sub = nh.subscribe(
        Config.points_topic, 1000,
        &Accumulator::receive_lidar, &accum
    );

    ros::Subscriber imu_sub = nh.subscribe(
        Config.imus_topic, 1000,
        &Accumulator::receive_imu, &accum
    );

    ros::Rate rate(Config.rate);
    
    while (ros::ok()) {
        if (accum.BUFFER_I.size() > 0) {
            // Custom time, one second before real time
            double t = accum.BUFFER_I.front().time - Config.real_time_delay; 

        MicroTimer compensator_clock;
        compensator_clock.tick();

            // Compensate pointcloud at time t
            PointCloud res = comp.compensate(t);
        
        compensator_clock.tock();
        ROS_INFO("Compensate: %f ms", compensator_clock.count());

            // Empty too old LiDARs
            accum.empty_lidar(t - Config.empty_lidar_time);

            publish.pointcloud(res);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}