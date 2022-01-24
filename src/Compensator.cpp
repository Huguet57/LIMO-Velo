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

// class Compensator
    // public:
        Points Compensator::compensate(double t1, double t2) {
            // Call Accumulator
            Accumulator& accum = Accumulator::getInstance();

            // Points from t1 to t2
            Points points = accum.get_points(t1, t2);
            if (points.empty()) return Points();

            // (Integrated) States surrounding t1 and t2
            States path_taken = this->path(t1, t2);
            assert (not path_taken.size() < 2);

            // ROS_INFO("Points: %d, path: %d", points.size(), path_taken.size());

            // Compensated pointcloud given a path
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
            IMUs imus = accum.get_imus(states.front().time, t2);
            imus.push_back(accum.get_next_imu(t2));

            return this->upsample(states, imus);
        }

    // private:
        
        State Compensator::get_t2(const States& states, double t2) {            
            int s = states.size();
            assert (t2 >= states.front().time);
            while (t2 >= states[--s].time);
            
            State Xt2 = states[s];
            Xt2 += IMU (Xt2.a, Xt2.w, t2);
            return Xt2;
        }

        /*
            New upsample.

                @Input:
                    states: before t1 and to t2
                    imus: before t1 and after t2
                
                @Output:
                    upsampled_states (size := imus.size): before t1 and after t2
        */
        States Compensator::upsample(const States& states, const IMUs& imus) {
            assert (imus.front().time <= states.front().time and states.back().time <= imus.back().time);

            int s, u;
            s = u = 0;

            States upsampled_states;
            State int_state = states[s];

            // IMUs between two states
            while (s < states.size() - 1) {
                while (u < imus.size() and int_state.time < states[s+1].time) {
                    int_state += imus[u++];
                    upsampled_states.push_back(int_state);
                }

                int_state = states[s++];
            }

            if (u >= imus.size()) u = imus.size() - 1;

            // IMUs after last state
            while (int_state.time < imus.back().time) {
                int_state += imus[u++];
                upsampled_states.push_back(int_state);
            }

            return upsampled_states;
        }

        PointCloud Compensator::downsample(const PointCloud& compensated) {
            return this->voxelgrid_downsample(compensated);
            // return this->onion_downsample(compensated);
        }

        /*
            New compensate.

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
            // States have to surround points
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

        PointCloud Compensator::voxelgrid_downsample(const PointCloud& compensated) {
            // Create a PointCloud pointer
            PointCloud::Ptr compensated_ptr(new PointCloud());
            *compensated_ptr = compensated;

            // Downsample using a VoxelGrid
            PointCloud downsampled_compensated;
            pcl::VoxelGrid<PointType> filter;
            filter.setInputCloud(compensated_ptr);
            filter.setLeafSize(0.5, 0.5, 0.5);
            filter.filter(downsampled_compensated);
            
            return downsampled_compensated;
        }

        PointCloud Compensator::onion_downsample(const PointCloud& pcl) {
            PointCloud downsampled_pcl;
            downsampled_pcl.header = pcl.header;
            downsampled_pcl.points.reserve(pcl.size()/4);

            for (int i = 0; i < pcl.size(); ++i) {
                PointType ptype = pcl.points[i];
                double range = Point (ptype).range;

                if (0 < range and range < 4 and (256/Config.ds_rate <= 1 or i%(256/Config.ds_rate) == 0)) downsampled_pcl.points.push_back(ptype);
                else if (4 < range and range < 6 and (64/Config.ds_rate <= 1 or i%(64/Config.ds_rate) == 0)) downsampled_pcl.points.push_back(ptype);
                else if (6 < range and range < 9 and (32/Config.ds_rate <= 1 or i%(32/Config.ds_rate) == 0)) downsampled_pcl.points.push_back(ptype);
                else if (9 < range and range < 12 and (16/Config.ds_rate <= 1 or i%(16/Config.ds_rate) == 0)) downsampled_pcl.points.push_back(ptype);
                else if (12 < range and range < 22 and (8/Config.ds_rate <= 1 or i%(8/Config.ds_rate) == 0)) downsampled_pcl.points.push_back(ptype);
                else if (22 < range and range < 30 and (4/Config.ds_rate <= 1 or i%(4/Config.ds_rate) == 0)) downsampled_pcl.points.push_back(ptype);
                else if (30 < range and range < 50 and (2/Config.ds_rate <= 1 or i%(2/Config.ds_rate) == 0)) downsampled_pcl.points.push_back(ptype);
                else if (range > 50) downsampled_pcl.points.push_back(ptype);
            }

            return downsampled_pcl;
        }