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
        PointCloud Compensator::compensate(double t1, double t2, bool global) {
            // Points from t1 to t2
            Points points = Accumulator::getInstance().get_points(t1, t2);
            if (points.empty()) return PointCloud();

            // (Integrated) States from t1 to t2
            States states = Accumulator::getInstance().get_states(t1, t2);
            IMUs imus = Accumulator::getInstance().get_imus(states.front().time, t2);
            States path_taken = this->integrate_imus(states, imus, t1, t2);

            // Compensated pointcloud given a path
            return this->compensate(path_taken, points, global);
        }

        States Compensator::integrate_imus(double t1, double t2) {
            // (Integrated) States from t1 to t2
            States states = Accumulator::getInstance().get_states(t1, t2);
            IMUs imus = Accumulator::getInstance().get_imus(states.front().time, t2);
            return this->integrate_imus(states, imus, t1, t2);
        }

    // private:
        
        // TODO: Use already propagated states in case there's no LiDAR (more efficiency)
        States Compensator::integrate_imus(States& states, const IMUs& imus, double t1, double t2) {
            if (states.size() == 0) states.push_back(State(imus.back().time));
            int Ip = Accumulator::getInstance().before_first_state(imus, states);
            States integrated_states;

            // We add a ghost state at t2
            States states_amp = states;
            states_amp.push_back(State(t2));

            for (int Xp = 0; Xp < states.size(); ++Xp) {
                State integrated_state = states[Xp];
                State next_state = states_amp[Xp+1];
                bool last_state = Xp == states.size() - 1;

                // Integrate up until the next known state
                while (Ip < imus.size() and integrated_state.time <= next_state.time) {
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

        // TODO: try to replicate the one from Fast-LIO2
        PointCloud Compensator::compensate(States& states, Points& points, bool global) {
            if (states.size() < 2) return PointCloud();

            // Get start and end states
            const State& Xt1 = states.front();
            const State& Xt2 = states.back();

            // Define state and point iterators
            int Xp = states.size() - 2;
            int Lp = points.size() - 1;

            // Compensate tj points to t2 frame
            PointCloud::Ptr result(new PointCloud());
            result->header.stamp = Conversions::sec2Microsec(Xt2.time);

            // Before and next known states
            // XtLp is the state we will integrate until point time
            State XtLp = states[Xp];
            State Xnow = states[Xp];
            State Xnext = Xt2;

            while (Xt1.time <= Xnow.time) {
                
                // Find all points with time > Xnow.time 
                while (0 <= Lp and Xnow.time < points[Lp].time) {

                    // Integrate up to point time
                    Point p_L_tLp = points[Lp--];
                    XtLp += IMU (XtLp.a, XtLp.w, p_L_tLp.time);

                    // Get rotation-translation pairs
                    RotTransl tnext_T_tLp = Xnext.inv() * XtLp;
                    RotTransl I_T_L = Xnext.I_Rt_L();

                    // Transport point to Xnext and add it
                    Point tnext_p_L_tLp = I_T_L.inv() * tnext_T_tLp * I_T_L * p_L_tLp;
                    if (not global) *result += tnext_p_L_tLp;         // LiDAR frame
                    else *result += Xnext * I_T_L * tnext_p_L_tLp;    // Global frame
                }

                // Iterate
                if (Xp > 0) {
                    if (global) Xnext = Xnow;
                    else Xnext = Xt2;
                    Xnow = states[--Xp];
                    XtLp = Xnow;
                } else {
                    break;
                }
            }

            return *result;
        }
