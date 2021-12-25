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

// class Compensator
    // public:
        PointCloud Compensator::compensate(double t1, double t2) {
            // Points from t1 to t2
            Points points = this->get_points(t1, t2);
            if (points.empty()) return PointCloud();

            // (Integrated) States from t1 to t2
            States states = this->get_states(t1, t2);
            IMUs imus = this->get_imus(states.front().time, t2);
            States path_taken = this->integrate_imus(states, imus, t1, t2);

            // Compensated pointcloud given a path
            return this->compensate(path_taken, points);
        }

    // private:
        IMU Compensator::get_next_imu(double t2) {
            IMU last_imu = this->BUFFER_Ip->front();
            for (IMU imu : this->BUFFER_Ip->content) {
                if (t2 >= imu.time) return last_imu;
                last_imu = imu;
            }

            // If not a state found, return the earliest (again)
            return this->BUFFER_Ip->front();
        }

        State Compensator::get_prev_state(double t1) {
            for (State x : this->BUFFER_Xp->content)
                if (t1 > x.time) return x;

            // If not a state found, push an empty one in t1
            this->BUFFER_Xp->push(State(t1));
            return this->BUFFER_Xp->front();
        }

        /////////////////////////////////

        States Compensator::get_states(double t1, double t2) {
            States states = this->get(this->BUFFER_Xp, t1, t2);
            states.push_front(this->get_prev_state(t1));
            return states;
        }

        Points Compensator::get_points(double t1, double t2) {
            return this->get(this->BUFFER_Lp, t1, t2);
        }

        IMUs Compensator::get_imus(double t1, double t2) {
            IMUs imus = this->get(this->BUFFER_Ip, t1, t2);
            imus.push_back(this->get_next_imu(t2));
            return imus;
        }

        ///////////////////////////////////////////////////

        // TODO: Use already propagated states in case there's no LiDAR (more efficiency)
        States Compensator::integrate_imus(States& states, const IMUs& imus, double t1, double t2) {
            if (states.size() == 0) states.push_back(State(imus.back().time));
            int Ip = this->before_first_state(imus, states);
            States integrated_states;

            // We add a ghost state at t2
            States states_amp = states;
            states_amp.push_back(State(t2));

            for (int Xp = 0; Xp < states.size(); ++Xp) {
                State integrated_state = states[Xp];
                State next_state = states_amp[Xp+1];
                bool last_state = Xp == states.size() - 1;

                // Integrate up until the next known state
                while (Ip < imus.size() and integrated_state.time < next_state.time) {
                    if (integrated_state.time >= t1)
                        integrated_states.push_back(integrated_state);
                    
                    if (last_state) integrated_state += imus[Ip++];
                    // TODO
                    // else integrated_state.interpolate(states, imus, Xp, Ip);
                    else integrated_state += imus[Ip++];
                }

                // Consider the case with t2-t1 < IMU sampling time
                if (last_state and integrated_states.size() == 0) {
                    integrated_states.push_back(integrated_state);
                }
            }
            
            return integrated_states;
        }

        PointCloud Compensator::compensate(States& states, Points& points) {
            // Get start and end states
            const State& Xt1 = states.front();
            const State& Xt2 = states.back();
            
            // Define state and point iterators
            int Xp = states.size() - 1;
            State& Xtj = states[Xp];
            int Lp = points.size() - 1;

            // Compensate tj points to t2 frame
            PointCloud::Ptr result(new PointCloud());
            result->header.stamp = Conversions::sec2Microsec(Xt2.time);

            while (Xt1.time <= Xtj.time) {
                // Get rotation-translation pairs
                RotTransl t2_T_tj = Xt2 - Xtj;
                RotTransl I_T_L = Xtj.offsets();

                // Find all points with time > Xtj.time 
                while (0 <= Lp and Xtj.time < points[Lp].time) {
                    Point p_L_tj = points[Lp--];
                    Point t2_p_L_tj = I_T_L.inv() * t2_T_tj * I_T_L * p_L_tj;

                    // *result += t2_p_L_tj;                       // LiDAR frame
                    *result += Xt2 * I_T_L * t2_p_L_tj;      // Global frame
                }

                // Depropagate state feeding IMUs in inverse order
                if (Xp > 0) Xtj = states[--Xp];
                else break;
            }

            return *result;
        }