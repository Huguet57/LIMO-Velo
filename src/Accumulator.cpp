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

        /////////////////////////////////

        IMU Accumulator::get_next_imu(double t2) {
            IMU last_imu = this->BUFFER_I.front();
            for (IMU imu : this->BUFFER_I.content) {
                if (t2 >= imu.time) return last_imu;
                last_imu = imu;
            }

            // If not a state found, return the earliest (again)
            return this->BUFFER_I.front();
        }

        State Accumulator::get_prev_state(double t1) {
            for (State x : this->BUFFER_X.content)
                if (t1 > x.time) return x;

            // If not a state found, push an empty one in t1
            this->BUFFER_X.push(State(t1));
            return this->BUFFER_X.front();
        }

        States Accumulator::get_states(double t1, double t2) {
            States states = this->get(this->BUFFER_X, t1, t2);
            states.push_front(this->get_prev_state(t1));
            return states;
        }

        Points Accumulator::get_points(double t1, double t2) {
            return this->get(this->BUFFER_L, t1, t2);
        }

        IMUs Accumulator::get_imus(double t1, double t2) {
            IMUs imus = this->get(this->BUFFER_I, t1, t2);
            imus.push_back(this->get_next_imu(t2));
            return imus;
        }

        //////////////////////////

        bool Accumulator::ready() {
            // Only check it once
            if (this->is_ready) return true;
            
            // Ready if there's enough IMUs to fit the delay
            if (this->enough_imus()) {
                this->set_initial_time();
                return this->is_ready = true;
            }

            return this->is_ready = false;
        }

        ros::Rate Accumulator::refine_delta(double t) {
            // Heuristic: every second, divide by 2 delta
            if (t - this->initial_time < 0.5) this->delta = 0.1; 
            else if (t - this->initial_time < 1.0) this->delta = 0.05; 
            else if (t - this->initial_time < 1.5) this->delta = 0.025; 
            else this->delta = 0.01; 
            
            return ros::Rate((int) 1./this->delta);
        }

    // private:

        bool Accumulator::enough_imus() {
            int IMU_RATE = 400;
            int Nimus = this->BUFFER_I.size();
            return Nimus > Config.real_time_delay*IMU_RATE;
        }

        void Accumulator::set_initial_time() {
            if (this->BUFFER_I.size() < 1) return;
            double latest_imu_time = this->BUFFER_I.front().time;

            this->initial_time = latest_imu_time - Config.real_time_delay;
        }
