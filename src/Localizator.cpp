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

// class Localizator
    // public:
        Localizator::Localizator() {
            this->map = &Mapper::getInstance();
            this->init_IKFoM();
        }

        Localizator::Localizator(Mapper* map) {
            this->map = map;
            this->init_IKFoM();
        }

        // Given points, update new position
        void Localizator::update(const PointCloud& points) {
            this->points2match = points;
            
            double solve_H_time = 0;
            this->IKFoM_update(solve_H_time);
        }

        void Localizator::calculate_H(const state_ikfom& s, const Planes& matches, Eigen::MatrixXd& H, Eigen::VectorXd& h) {
            // int Nmatches = matches.size();
            // H = Eigen::MatrixXd::Zero(Nmatches, 12);
            // h.resize(Nmatches);

            // // For each match, calculate its derivative and distance
            // for (const Plane& match : matches) {
            //     Normal n = match.n;
            //     Point p_lidar_x = Point (n.x, n.y, n.z);

            //     // Calculate H (:= dh/dx)
            //     V3D A = (s.offset_R_L_I * p_lidar_x).cross(s.rot.conjugate() * n);
            //     V3D B = (p_lidar_x).cross(s.offset_R_L_I.conjugate() * s.rot.conjugate() * n);
            //     V3D C = (s.rot.conjugate() * n);
                
            //     H.block<1, 6>(i,0) << n.A, n.B, n.C, VEC_FROM_ARRAY(A);
            //     if (extr_est) H.block<1, 6>(i,6) << VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);

            //     // Measurement: distance to the closest plane
            //     h(i) = -match.distance;
            // }
        }

    // private:
        Localizator& Localizator::getInstance() {
            Mapper& map = Mapper::getInstance();
            static Localizator* localizator = new Localizator(&map);
            return *localizator;
        }

        void Localizator::init_IKFoM() {
            // Constants
            int MAX_NUM_ITERS = 5;
            Eigen::VectorXf LIMITS = 0.001*Eigen::VectorXf::Ones(23);

            // Initialize IKFoM
            this->KF.init_dyn_share(
                // TODO: change to private functions instead of "IKFoM::"
                IKFoM::get_f,
                IKFoM::df_dx,
                IKFoM::df_dw,
                IKFoM::h_share_model,

                MAX_NUM_ITERS,
                LIMITS
            );
        }

        void Localizator::IKFoM_update(double& solve_H_time) {
            this->KF.update_iterated_dyn_share_modified(0.001, solve_H_time);
        }