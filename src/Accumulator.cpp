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
        // Add content to buffer
        void Accumulator::add(State cnt, double time) {
            if (time > 0) cnt.time = time;
            this->push(cnt);
        }

        void Accumulator::add(IMU cnt, double time) {
            if (time > 0) cnt.time = time;
            this->push(cnt);
        }

        void Accumulator::add(Point cnt, double time) {
            if (time > 0) cnt.time = time;
            this->push(cnt);
        }

        void Accumulator::add(Points points) {
            for (Point p : points) this->push(p);
        }

        // Receive from topics
        void Accumulator::receive_lidar(const PointCloud_msg& msg) {
            PointCloudProcessor processed(msg);
            this->add(processed.points);
        }

        void Accumulator::receive_imu(const IMU_msg& msg) {
            IMU imu(msg);
            this->add(imu);
        }

        // Empty buffers
        void Accumulator::clear_buffers() {
            this->BUFFER_L.clear();
            this->BUFFER_I.clear();
        }

        void Accumulator::clear_buffers(TimeType t) {
            this->BUFFER_L.clear(t);
            this->BUFFER_I.clear(t);
        }

        void Accumulator::clear_lidar(TimeType t) {
            this->BUFFER_L.clear(t);
        }

        /////////////////////////////////

        States Accumulator::get_states(double t1, double t2) {
            States states = this->get(this->BUFFER_X, t1, t2);
            states.push_front(this->get_prev(this->BUFFER_X, t1));
            return states;
        }

        Points Accumulator::get_points(double t1, double t2) {
            return this->get(this->BUFFER_L, t1, t2);
        }

        IMUs Accumulator::get_imus(double t1, double t2) {
            IMUs imus = this->get(this->BUFFER_I, t1, t2);
            imus.push_back(this->get_next(this->BUFFER_I, t2));
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

        bool Accumulator::ended(double t) {
            if (t - initial_time < 3) return false;
            return this->ready() and this->get_imus(t - 3., t).size() < 2;
        }

        ros::Rate Accumulator::refine_delta(const HeuristicParams& heuristic, double t) {
            assert(("There has to be exactly one more delta value than time delimiters", heuristic.times.size() + 1 == heuristic.deltas.size()));
            this->delta = this->interpret_heuristic(heuristic, t);
            return ros::Rate((int) 1./this->delta);
        }

    // private:

        void Accumulator::push(const State& state) { this->BUFFER_X.push(state); }
        void Accumulator::push(const IMU& imu) { this->BUFFER_I.push(imu); }
        void Accumulator::push(const Point& point) { this->BUFFER_L.push(point); }

        bool Accumulator::enough_imus() {
            return this->BUFFER_I.size() > Config.real_time_delay*Config.imu_rate;
        }

        void Accumulator::set_initial_time() {
            if (this->BUFFER_I.size() < 1) return;
            double latest_imu_time = this->BUFFER_I.front().time;

            this->initial_time = latest_imu_time - Config.real_time_delay;
        }

        double Accumulator::interpret_heuristic(const HeuristicParams& heuristic, double t) {
            // If is after last time
            if (heuristic.times.empty()) return heuristic.deltas.back();
            if (t - this->initial_time >= heuristic.times.back()) return heuristic.deltas.back();
            
            // If we have to find it in the list
            for (int k = 0; k < heuristic.times.size(); ++k)
                if (t - this->initial_time < heuristic.times[k])
                    return heuristic.deltas[k];

            return heuristic.deltas.back();
        }