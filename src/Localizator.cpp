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
            this->init_IKFoM();
        }

        // Given points, update new position
        void Localizator::update(const PointCloud& points) {
            this->points2match = points;
            
            double solve_H_time = 0;
            this->IKFoM_update(solve_H_time);
        }

        void Localizator::calculate_H(const state_ikfom& s, const Planes& matches, Eigen::MatrixXd& H, Eigen::VectorXd& h) {
            int Nmatches = matches.size();
            H = Eigen::MatrixXd::Zero(Nmatches, 12);
            h.resize(Nmatches);

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

        void Localizator::propagate_to(double t) {
            if (this->last_time_integrated < 0) this->last_time_integrated = t;
            IMUs imus = Accumulator::getInstance().get_imus(this->last_time_integrated, t);
            for (IMU imu : imus) this->propagate(imu);
        }

        State Localizator::latest_state() {
            return State(
                this->get_x(),
                this->last_time_integrated
            );
        }

    // private:
        Localizator& Localizator::getInstance() {
            static Localizator* localizator = new Localizator();
            return *localizator;
        }

        void Localizator::init_IKFoM() {
            // Constants
            int MAX_NUM_ITERS = 5;
            Eigen::VectorXf LIMITS = 0.001*Eigen::VectorXf::Ones(23);

            // Initialize IKFoM
            this->IKFoM_KF.init_dyn_share(
                // TODO: change to private functions instead of "IKFoM::"
                IKFoM::get_f,
                IKFoM::df_dx,
                IKFoM::df_dw,
                IKFoM::h_share_model,

                MAX_NUM_ITERS,
                LIMITS
            );

            // Initialize state
            this->init_IKFoM_state();
        }

        void Localizator::IKFoM_update(double& solve_H_time) {
            this->IKFoM_KF.update_iterated_dyn_share_modified(0.001, solve_H_time);
        }

        void Localizator::init_IKFoM_state() {
            state_ikfom init_state = this->IKFoM_KF.get_x();
            init_state.grav = S2(Eigen::Vector3d (0.,0.,+9.807));
            init_state.bg  = Eigen::Vector3d(0.,0.,0.);
            init_state.offset_R_L_I = SO3(Eigen::Matrix3d(Eigen::Vector3d(1,-1,-1).asDiagonal()));
            init_state.offset_T_L_I = Eigen::Vector3d(0.9, 0., 0.);
            this->IKFoM_KF.change_x(init_state);

            esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = this->IKFoM_KF.get_P();
            init_P.setIdentity();
            init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
            init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
            init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
            init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
            init_P(21,21) = init_P(22,22) = 0.00001; 
            this->IKFoM_KF.change_P(init_P);
        }

        const state_ikfom& Localizator::get_x() const {
            return this->IKFoM_KF.get_x();
        }

        void Localizator::propagate(const IMU& imu) {
            input_ikfom in;
            in.acc = imu.a.cast<double>();
            in.gyro = imu.w.cast<double>();

            Eigen::Matrix<double, 12, 12> Q;
            Q.block<3, 3>(0, 0).diagonal() = Eigen::Vector3d::Constant(6.01e-4); // cov_gyr;
            Q.block<3, 3>(3, 3).diagonal() = Eigen::Vector3d::Constant(1.53e-2); // cov_acc;
            Q.block<3, 3>(6, 6).diagonal() = Eigen::Vector3d::Constant(3.38e-4); // cov_bias_gyr;
            Q.block<3, 3>(9, 9).diagonal() = Eigen::Vector3d::Constant(1.54e-5); // cov_bias_acc;

            double dt = imu.time - this->last_time_integrated;
            this->last_time_integrated = imu.time;

            this->IKFoM_KF.predict(dt, Q, in);
        }