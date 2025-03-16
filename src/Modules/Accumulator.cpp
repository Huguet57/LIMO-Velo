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
            void Accumulator::receive_lidar(const PointCloud_msg msg) {
                // Turn message to processed points
                Points points = this->process(msg);
                
                // Check if missing data
                if (this->missing_data(points)) this->throw_warning(points);

                // Add them individually on the LiDAR buffer
                for (Point p : points) this->add(p);
            }

            void Accumulator::receive_state(const State_msg msg) {
                // Turn message to IMU object
                auto imu_msg = sensor_msgs::msg::Imu();

                imu_msg.header.stamp = msg.header.stamp;
                imu_msg.linear_acceleration.x = msg.ax;
                imu_msg.linear_acceleration.y = msg.ay;
                imu_msg.angular_velocity.z = msg.r;

                // Add it to the IMU buffer
                this->add(imu_msg);
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

        State Accumulator::get_prev_state(double t) {
            if (this->BUFFER_X.empty()) {
                State X = Localizator::getInstance().latest_state();
                Accumulator::getInstance().add(X, t);
                X.time = t;
                return X;
            }
            
            return this->get_prev(this->BUFFER_X, t);
        }

        IMU Accumulator::get_next_imu(double t) {
            return this->get_next(this->BUFFER_I, t);
        }

        States Accumulator::get_states(double t1, double t2) {
            return this->get(this->BUFFER_X, t1, t2);
        }

        Points Accumulator::get_points(double t1, double t2) {
            return this->get(this->BUFFER_L, t1, t2);
        }

        IMUs Accumulator::get_imus(double t1, double t2) {
            return this->get(this->BUFFER_I, t1, t2);
        }

        //////////////////////////

        bool Accumulator::ready() {
            // Only check it once
            if (this->is_ready) return true;
            
            // Debugging checks
            if (Config.print_debug) {
                RCLCPP_INFO(rclcpp::get_logger("limovelo"), "BUFFER_I size: %ld", this->BUFFER_I.size());
                RCLCPP_INFO(rclcpp::get_logger("limovelo"), "BUFFER_L size: %ld", this->BUFFER_L.size());
                RCLCPP_INFO(rclcpp::get_logger("limovelo"), "BUFFER_X size: %ld", this->BUFFER_X.size());
            }

            // Ready if there's enough IMUs to fit the delay
            if (this->enough_imus()) {
                this->set_initial_time();
                Localizator::getInstance().initialize(this->initial_time);
                if (Config.print_debug) RCLCPP_INFO(rclcpp::get_logger("limovelo"), "Accumulator is now ready");
                return this->is_ready = true;
            } else {
                if (Config.print_debug) RCLCPP_WARN(rclcpp::get_logger("limovelo"), "Not enough IMUs yet");
            }

            return this->is_ready = false;
        }

        // Check if data stream died
        bool Accumulator::ended(double t) {
            if (not this->ready()) return false;
            if (t - initial_time < 3) return false;
            return this->get_imus(t - 3., t).size() < 2;
        }

        double Accumulator::update_delta(const InitializationParams& initialization, double t) {
            assert(("There has to be exactly one more delta value than time delimiters", initialization.times.size() + 1 == initialization.deltas.size()));
            return this->interpret_initialization(initialization, t);
        }

        double Accumulator::latest_time() {
            // // Ideally should be ros::Time::now() - delay, but it's easier the other way with rosbags (no need for use_sim_time=true)
            // return ros::Time::now() - Config.real_time_delay;

            // Latest IMU timestamp - delay
            return this->BUFFER_I.front().time - Config.real_time_delay;
        }

    // private:

        void Accumulator::push(const State& state) { this->BUFFER_X.push(state); }
        void Accumulator::push(const IMU& imu) { this->BUFFER_I.push(imu); }
        void Accumulator::push(const Point& point) { this->BUFFER_L.push(point); }

        Points Accumulator::process(const PointCloud_msg& msg) {
            // Create a temporal object to process the pointcloud message
            PointCloudProcessor processor;
            Points points = processor.msg2points(msg);
            Points downsampled_points = processor.downsample(points);
            Points sorted_points = processor.sort_points(downsampled_points);
            
            // Return sorted (downsampled) points as "processed points"
            return sorted_points;
        }

        bool Accumulator::enough_imus() {
            return this->BUFFER_I.size() > Config.real_time_delay*Config.imu_rate;
        }

        void Accumulator::set_initial_time() {
            if (this->BUFFER_I.size() < 1) return;
            double latest_imu_time = this->BUFFER_I.front().time;

            this->initial_time = latest_imu_time - Config.real_time_delay;
        }

        double Accumulator::interpret_initialization(const InitializationParams& initialization, double t) {
            // If is after last time
            if (initialization.times.empty()) return initialization.deltas.back();
            if (t - this->initial_time >= initialization.times.back()) return initialization.deltas.back();
            
            // If we have to find it in the list
            for (int k = 0; k < initialization.times.size(); ++k)
                if (t - this->initial_time < initialization.times[k])
                    return initialization.deltas[k];

            return initialization.deltas.back();
        }

        bool Accumulator::missing_data(const Points& time_sorted_points) {
            if (time_sorted_points.size() < Config.MAX_POINTS2MATCH) return false;
            
            // Check missing 'time' information
            if (time_sorted_points.front().time == 0 and time_sorted_points.back().time == 0) {
                // Remove Initialization
                Config.Initialization.times = {};
                Config.Initialization.deltas = {Config.full_rotation_time};
                
                return true;
            }

            return false;
        }

        void Accumulator::throw_warning(const Points& time_sorted_points) {
            // Warn once
            if (not this->has_warned_lidar) this->has_warned_lidar = true;
            else return;

            // Warn missing 'time' information
            RCLCPP_ERROR(rclcpp::get_logger("limovelo"), "LiDAR points are missing 'time' information.");
            RCLCPP_ERROR(rclcpp::get_logger("limovelo"), "Delta has been fixed to %f (s) leading to a fixed %d (Hz) localization.", Config.full_rotation_time, (int) 1./Config.full_rotation_time);
        }