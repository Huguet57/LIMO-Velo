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

extern struct Params Config;

// class IMU {
    // public:
        IMU::IMU() : IMU::IMU (0.) {}
        IMU::IMU(const sensor_msgs::msg::Imu::ConstPtr& msg) : IMU::IMU(*msg) {}

        IMU::IMU(const sensor_msgs::msg::Imu& imu) {
            // Linear accelerations
            this->a(0) = imu.linear_acceleration.x;
            this->a(1) = imu.linear_acceleration.y;
            this->a(2) = imu.linear_acceleration.z;

            // Gyroscope
            this->w(0) = imu.angular_velocity.x;
            this->w(1) = imu.angular_velocity.y;
            this->w(2) = imu.angular_velocity.z;

            // Orientation
            this->q.x() = imu.orientation.x;
            this->q.y() = imu.orientation.y;
            this->q.z() = imu.orientation.z;
            this->q.w() = imu.orientation.w;

            // Time
            this->time = Conversions::nanosec2Sec(imu.header.stamp.nanosec) + imu.header.stamp.sec;
        }

        IMU::IMU (const Eigen::Vector3f& a, const Eigen::Vector3f& w, double time) : IMU::IMU(a, w, Eigen::Quaternionf (1,0,0,0), time) {
            this->a = a;
            this->w = w;
            this->q = q;
            this->time = time;
        }

        IMU::IMU (const Eigen::Vector3f& a, const Eigen::Vector3f& w, const Eigen::Quaternionf& q, double time) {
            this->a = a;
            this->w = w;
            this->q = q;
            this->time = time;
        }

        IMU::IMU (double time) : IMU::IMU (Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Quaternionf (1,0,0,0), time) {}

        bool IMU::has_orientation() {
            return not (this->q.x() == 0 and this->q.y() == 0 and this->q.z() == 0 and this->q.w() == 0);
        }