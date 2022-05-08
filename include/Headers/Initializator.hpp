#ifndef SLAM_LIBRARIES_H
#define SLAM_LIBRARIES_H
// SLAM libraries
#include "use-ikfom.hpp"
#include "ikd_Tree.h"
#endif

extern struct Params Config;

class Initializator {
    public:
        State initial_S;

        bool done() const {
            return this->_done;
        }

        State read_extrinsics() {
            // Read YAML parameters
            this->initial_S.g = Eigen::Map<Eigen::Vector3f>(Config.initial_gravity.data(), 3);
            this->initial_S.tLI = Eigen::Map<Eigen::Vector3f>(Config.I_Translation_L.data(), 3);
            this->initial_S.RLI = Eigen::Map<Eigen::Matrix3f>(Config.I_Rotation_L.data(), 3, 3).transpose();
            
            return this->initial_S;
        }

        State set_initial_pos(const Eigen::Vector3f& pos, const Eigen::Matrix3f& R) {
            // Set position and orientation
            this->initial_S.pos = pos;
            this->initial_S.R = R;

            return this->initial_S;
        }

        State set_initial_IMU(double time) {
            // Set time
            this->initial_S.time = time;

            // Set IMU
            IMU imu = Accumulator::getInstance().get_imus(-1, time).back();
            this->initial_S.a = imu.a;
            this->initial_S.w = imu.w;
            
            if (imu.has_orientation())
                this->initial_S.R = imu.q.toRotationMatrix();

            return this->initial_S;
        }

    private:
        bool _done = false;

    // Singleton pattern
    public:
        static Initializator& getInstance() {
            static Initializator* init = new Initializator();
            return *init;
        }

    private:
        Initializator() = default;

        // Delete copy/move so extra instances can't be created/moved.
        Initializator(const Initializator&) = delete;
        Initializator& operator=(const Initializator&) = delete;
        Initializator(Initializator&&) = delete;
        Initializator& operator=(Initializator&&) = delete;
};