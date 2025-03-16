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

// class Compensator
    // public:
        Points Compensator::compensate(double t1, double t2) {
            // Call Accumulator
            Accumulator& accum = Accumulator::getInstance();

            // Points from t1 to t2
            Points points = accum.get_points(t1, t2);
            if (Config.print_debug) RCLCPP_INFO(rclcpp::get_logger("limovelo"), "Points: %ld", points.size());
            if (points.empty()) {
                RCLCPP_WARN(rclcpp::get_logger("limovelo"), "No points to compensate");
                return Points();
            };

            // (Integrated) States surrounding t1 and t2
            States path_taken = this->path(t1, t2);
            assert (path_taken.size() >= 2);

            // Compensated points given a path
            State Xt2 = this->get_t2(path_taken, t2);

            return this->compensate(path_taken, Xt2, points);
        }

        States Compensator::path(double t1, double t2) {
            // Call Accumulator
            Accumulator& accum = Accumulator::getInstance();

            // Get states just before t1 to t2
            States states = accum.get_states(t1, t2);
            states.push_front(accum.get_prev_state(t1));

            // Get imus from first state to just after t2
            if (Config.print_debug) RCLCPP_INFO(rclcpp::get_logger("limovelo"), "t1: %f", t1);
            if (Config.print_debug) RCLCPP_INFO(rclcpp::get_logger("limovelo"), "t2: %f", t2);
            if (Config.print_debug) RCLCPP_INFO(rclcpp::get_logger("limovelo"), "states.size(): %ld", states.size());
            IMUs imus = accum.get_imus(states.front().time, t2);
            if (Config.print_debug) RCLCPP_INFO(rclcpp::get_logger("limovelo"), "imus.size(): %ld", imus.size());
            imus.push_back(accum.get_next_imu(t2));

            return this->upsample(states, imus);
        }

    // private:
        
        State Compensator::get_t2(const States& states, double t2) {            
            int s = states.size() - 1;
            assert (states.front().time <= t2);
            while (t2 < states[s].time) --s;
            
            State Xt2 = states[s];
            Xt2 += IMU (Xt2.a, Xt2.w, t2);
            return Xt2;
        }

        /*
            @Input:
                states: before t1 and to t2
                imus: before t1 and after t2
            
            @Output:
                upsampled_states (size := imus.size): before t1 and after t2
        */
        States Compensator::upsample(const States& states, const IMUs& imus) {
            //assert (imus.front().time <= states.front().time and states.back().time <= imus.back().time);

            int s, u;
            s = u = 0;

            States upsampled_states;
            State int_state = states[s];

            // IMUs between two states
            while (s < states.size() - 1) {
                upsampled_states.push_back(states[s]);

                while (u < imus.size() and imus[u].time < states[s+1].time) {
                    int_state += imus[u++];
                    upsampled_states.push_back(int_state);
                }

                int_state = states[s++];
            }

            if (u >= imus.size()) u = imus.size() - 1;
            upsampled_states.push_back(states.back());
            int_state = states.back();

            // IMUs after last state
            while (int_state.time < imus.back().time and u < imus.size()) {
                int_state += imus[u++];
                upsampled_states.push_back(int_state);
            }

            return upsampled_states;
        }

        Points Compensator::downsample(const Points& points) {
            return this->voxelgrid_downsample(points);
            // return this->onion_downsample(points);
        }

        /*
            @Input:
                states: path the car has taken (pre and post included)
                points: stamped points during path
            @Output:
                compensated_points: compensated points ready to be transported by Xt2
            
            @Pseudocode:
                for each state:
                    for each point between state and next_state:
                        compensate point matching its time via integrating state's last IMU
        */

        Points Compensator::compensate(const States& states, const State& Xt2, const Points& points) {
            if (states.empty()) {
                RCLCPP_WARN(rclcpp::get_logger("limovelo"), "Compensator::compensate: States vector is empty, skipping compensation.");
                return points;
            }
            assert (not states.empty() and states.front().time <= points.front().time and  points.back().time <= states.back().time);

            Points t2_inv_ps;
            int p = 0;

            for (int s = 0; s < states.size() - 1; ++s) {
                while (p < points.size() and states[s].time <= points[p].time and points[p].time <= states[s+1].time) {                    
                    // Integrate to point time
                    State Xtp = states[s];
                    Xtp += IMU (states[s].a, states[s].w, points[p].time);

                    // Transport to X_t2^-1 frame
                    Point global_p = Xtp * Xtp.I_Rt_L() * points[p];
                    Point t2_inv_p = Xt2.I_Rt_L().inv() * Xt2.inv() * global_p;
                    t2_inv_ps.push_back(t2_inv_p);

                    ++p;
                }
            }

            return t2_inv_ps;
        }

        Points Compensator::voxelgrid_downsample(const Points& points) {
            // Create a PointCloud pointer
            pcl::PointCloud<full_info::Point>::Ptr pcl_ptr(new pcl::PointCloud<full_info::Point>());
            Processor::fill(*pcl_ptr, points);

            // Downsample using a VoxelGrid
            pcl::PointCloud<full_info::Point> ds_pcl;
            pcl::VoxelGrid<full_info::Point> filter;
            filter.setInputCloud(pcl_ptr);
            filter.setLeafSize(Config.downsample_prec, Config.downsample_prec, Config.downsample_prec);
            filter.filter(ds_pcl);
            
            Points ds_points;
            for (auto p : ds_pcl.points) ds_points.push_back(Point (p));
            return ds_points;
        }

        Points Compensator::onion_downsample(const Points& points) {
            Points ds_points;

            for (int i = 0; i < points.size(); ++i) {
                const Point& p = points[i];
                if (0 < p.range and p.range < 4 and (256/Config.downsample_rate <= 1 or i%(256/Config.downsample_rate) == 0)) ds_points.push_back(p);
                else if (4 < p.range and p.range < 6 and (64/Config.downsample_rate <= 1 or i%(64/Config.downsample_rate) == 0)) ds_points.push_back(p);
                else if (6 < p.range and p.range < 9 and (32/Config.downsample_rate <= 1 or i%(32/Config.downsample_rate) == 0)) ds_points.push_back(p);
                else if (9 < p.range and p.range < 12 and (16/Config.downsample_rate <= 1 or i%(16/Config.downsample_rate) == 0)) ds_points.push_back(p);
                else if (12 < p.range and p.range < 22 and (8/Config.downsample_rate <= 1 or i%(8/Config.downsample_rate) == 0)) ds_points.push_back(p);
                else if (22 < p.range and p.range < 30 and (4/Config.downsample_rate <= 1 or i%(4/Config.downsample_rate) == 0)) ds_points.push_back(p);
                else if (30 < p.range and p.range < 50 and (2/Config.downsample_rate <= 1 or i%(2/Config.downsample_rate) == 0)) ds_points.push_back(p);
                else if (p.range > 50) ds_points.push_back(p);
            }

            return ds_points;
        }