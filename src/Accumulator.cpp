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

// class Accumulator
    // public:
        // Receive from topics
        void Accumulator::receive_lidar(const PointCloud_msg& msg) {
            PointCloudProcessor processed(msg, this->BUFFER_L);
        }

        void Accumulator::receive_imu(const IMU_msg& msg) {
            IMU imu(msg);
            this->BUFFER_I.push(imu);
        }

        // Empty buffers
        void Accumulator::empty_buffers() {
            this->BUFFER_L.empty();
            this->BUFFER_I.empty();
        }

        void Accumulator::empty_buffers(TimeType t) {
            this->BUFFER_L.empty(t);
            this->BUFFER_I.empty(t);
        }

        void Accumulator::empty_lidar(TimeType t) {
            this->BUFFER_L.empty(t);
        }