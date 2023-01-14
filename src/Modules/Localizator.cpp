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

        // Given points, find new position
        void Localizator::correct(const Points& points, double time) {
            if (not Mapper::getInstance().exists()) return;
            this->IKFoM_update(points);
            this->last_time_updated = time;
        }

        void Localizator::calculate_H(const state_ikfom& s, const Matches& matches, Eigen::MatrixXd& H, Eigen::VectorXd& h) {
            int Nmatches = matches.size();
            H = Eigen::MatrixXd::Zero(Nmatches, 12);
            h.resize(Nmatches);
            State S(s, 0.);

            // For each match, calculate its derivative and distance
            for (int i = 0; i < matches.size(); ++i) {
                Match match = matches[i];
                Point p_lidar = S.I_Rt_L().inv() * S.inv() * match.point;
                Point p_imu = S.I_Rt_L() * p_lidar;
                Normal n = match.plane.n;

                // Rotation matrices
                Eigen::Matrix3d R_inv = s.rot.conjugate().toRotationMatrix();
                Eigen::Matrix3d I_R_L_inv = s.offset_R_L_I.conjugate().toRotationMatrix();

                // Calculate H (:= dh/dx)
                Eigen::Vector3d C = (R_inv * n);
                Eigen::Vector3d B = (p_lidar).cross(I_R_L_inv * C);
                Eigen::Vector3d A = (p_imu).cross(C);
                
                H.block<1, 6>(i,0) << n.A, n.B, n.C, A(0), A(1), A(2);
                if (Config.estimate_extrinsics) H.block<1, 6>(i,6) << B(0), B(1), B(2), C(0), C(1), C(2);

                // Measurement: distance to the closest plane
                h(i) = -match.distance;
            }
        }

        void Localizator::propagate_to(double t) {
            // Get new IMUs
            IMUs imus = Accumulator::getInstance().get_imus(this->last_time_integrated, t);
            if (this->last_time_integrated < 0) this->last_time_integrated = t;

            // Integrate every new IMU between last time and now
            for (IMU imu : imus) {
                this->propagate(imu);
                this->last_time_integrated = imu.time;
            }
            
            // Propagate last known IMU to t 
            if (not imus.empty()) {
                this->propagate(IMU (imus.back().a, imus.back().w, t));
                this->last_time_integrated = t;
            }
        }

        State Localizator::latest_state() {
            // If no integrated, return empty state
            if (this->last_time_integrated < 0)
                return State (
                    this->get_x(),
                    Accumulator::getInstance().initial_time
                );
            
            // If no updates, return integrated state
            if (this->last_time_updated < 0)
                return State (
                    this->get_x(),
                    this->last_time_integrated
                );
            
            // Otherwise, return corrected state
            return State (
                this->get_x(),
                this->last_time_updated
            );
        }

    // private:
        Localizator& Localizator::getInstance() {
            static Localizator* localizator = new Localizator();
            return *localizator;
        }

        void Localizator::init_IKFoM() {
            // Initialize IKFoM
            this->IKFoM_KF.init_dyn_share(
                // TODO: change to private functions instead of "IKFoM::"
                IKFoM::get_f,
                IKFoM::df_dx,
                IKFoM::df_dw,
                IKFoM::h_share_model,

                Config.MAX_NUM_ITERS,
                Config.LIMITS
            );
        }

        void Localizator::initialize(double t) {
            // Get IMU at t
            IMUs imus = Accumulator::getInstance().get_imus(-1, t);
            IMU initial_IMU = imus.back();

            // Initialize state
            this->init_IKFoM_state(initial_IMU);
            this->initialized = true;
        }

        void Localizator::IKFoM_update(const Points& points) {
            double solve_H_time = 0;
            this->points2match = points;            
            this->IKFoM_KF.update_iterated_dyn_share_modified(Config.LiDAR_noise, Config.degeneracy_threshold, solve_H_time, Config.print_degeneracy_values);
        }

        void Localizator::init_IKFoM_state(const IMU& imu) {
            state_ikfom init_state = this->IKFoM_KF.get_x();
            init_state.rot = imu.q.cast<double> ();
            init_state.grav = S2(-Eigen::Vector3f (Conversions::double2floatVect(Config.initial_gravity).data()).cast<double>());
            init_state.bg = Eigen::Vector3d::Zero();
            init_state.offset_R_L_I = SO3(Eigen::Map<Eigen::Matrix3f>(Conversions::double2floatVect(Config.I_Rotation_L).data(), 3, 3).cast<double>());
            init_state.offset_T_L_I = Eigen::Vector3f(Conversions::double2floatVect(Config.I_Translation_L).data()).cast<double>();
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

            Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Identity();
            Q.block<3, 3>(0, 0) = Config.cov_gyro * Eigen::Matrix<double, 3, 3>::Identity();
            Q.block<3, 3>(3, 3) = Config.cov_acc * Eigen::Matrix<double, 3, 3>::Identity();
            Q.block<3, 3>(6, 6) = Config.cov_bias_gyro * Eigen::Matrix<double, 3, 3>::Identity();
            Q.block<3, 3>(9, 9) = Config.cov_bias_acc * Eigen::Matrix<double, 3, 3>::Identity();

            double dt = imu.time - this->last_time_integrated;

            this->IKFoM_KF.predict(dt, Q, in);
        }

        void Localizator::set_orientation(const IMU& imu) {
            state_ikfom current_state = this->IKFoM_KF.get_x();
            current_state.rot = imu.q.cast<double>();
            this->IKFoM_KF.change_x(current_state);
        }
        