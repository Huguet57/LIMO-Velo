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

// class Localizator
    // public:
        Localizator::Localizator() {
            this->init_IKFoM();
        }

        // Given points, update new position
        void Localizator::update(const PointCloud& points) {
            if (not Mapper::getInstance().exists()) return;
            this->points2match = points;
            
            double solve_H_time = 0;
            this->IKFoM_update(solve_H_time);
        }

        void Localizator::calculate_H(const state_ikfom& s, const Planes& matches, Eigen::MatrixXd& H, Eigen::VectorXd& h) {
            int Nmatches = matches.size();
            H = Eigen::MatrixXd::Zero(Nmatches, 12);
            h.resize(Nmatches);

            // For each match, calculate its derivative and distance
            for (int i = 0; i < matches.size(); ++i) {
                Plane match = matches[i];
                Point p_lidar_x = match.centroid;
                Normal n = match.n;

                // Rotation matrices
                State S(s, 0.);
                Eigen::Matrix3d R = s.rot.toRotationMatrix();
                Eigen::Matrix3d Rinv = s.rot.conjugate().toRotationMatrix();
                Eigen::Matrix3d R_L_I = s.offset_R_L_I.toRotationMatrix();
                Eigen::Matrix3f R_L_If = R_L_I.template cast<float>();

                // Calculate H (:= dh/dx)
                Eigen::Vector3d C = (Rinv * n);
                Eigen::Vector3d B = (p_lidar_x).cross(R_L_I.transpose() * C);
                Eigen::Vector3d A = (S.I_Rt_L() * p_lidar_x).cross(C);
                
                bool extr_est = false;
                H.block<1, 6>(i,0) << n.A, n.B, n.C, A(0), A(1), A(2);
                if (extr_est) H.block<1, 6>(i,6) << B(0), B(1), B(2), C(0), C(1), C(2);

                // Measurement: distance to the closest plane
                Point p = S * S.I_Rt_L() * match.centroid;
                double distance = n.A * p.x + n.B * p.y + n.C * p.z + n.D;
                h(i) = -distance;
            }
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
            Eigen::VectorXf LIMITS = 0.001*Eigen::VectorXf::Ones(23);

            // Initialize IKFoM
            this->IKFoM_KF.init_dyn_share(
                // TODO: change to private functions instead of "IKFoM::"
                IKFoM::get_f,
                IKFoM::df_dx,
                IKFoM::df_dw,
                IKFoM::h_share_model,

                Config.MAX_NUM_ITERS,
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
            Q.setIdentity();

            Q(0,0) = 6.01e-4;
            Q(1,1) = 6.01e-4;
            Q(2,2) = 6.01e-4;
            
            Q(3,3) = 1.53e-2;
            Q(4,4) = 1.53e-2;
            Q(5,5) = 1.53e-2;
            
            Q(6,6) = 3.38e-4;
            Q(7,7) = 3.38e-4;
            Q(8,8) = 3.38e-4;
            
            Q(9,9) = 1.54e-5;
            Q(10,10) = 1.54e-5;
            Q(11,11) = 1.54e-5;
            
            // Q.block<3, 3>(0, 0).diagonal() = Eigen::Vector3d::Constant(6.01e-4); // cov_gyr;
            // Q.block<3, 3>(3, 3).diagonal() = Eigen::Vector3d::Constant(1.53e-2); // cov_acc;
            // Q.block<3, 3>(6, 6).diagonal() = Eigen::Vector3d::Constant(3.38e-4); // cov_bias_gyr;
            // Q.block<3, 3>(9, 9).diagonal() = Eigen::Vector3d::Constant(1.54e-5); // cov_bias_acc;

            double dt = imu.time - this->last_time_integrated;
            this->last_time_integrated = imu.time;

            this->IKFoM_KF.predict(dt, Q, in);
        }