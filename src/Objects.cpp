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

// class Buffer
template class Buffer<IMU>;
template class Buffer<Point>;
template class Buffer<State>;

// class Point
    // public:
        Point operator*(const Eigen::Matrix<float, 3, 3> M, const Point& p) {
            Eigen::Matrix<float, 3, 1> moved_p = M*p.toEigen();
            return Point(moved_p, p.time);
        }

        Point operator+(const Point& p, const Eigen::Matrix<float, 3, 1> v) {
            Eigen::Matrix<float, 3, 1> moved_p = p.toEigen() + v;
            return Point(moved_p, p.time);
        }

        Point operator-(const Point& p, const Eigen::Matrix<float, 3, 1> v) {
            return p + (-v);
        }

        std::ostream& operator<< (std::ostream& out, const Point& p) {
            out << "(x="
                << p.x << ", y="
                << p.y << ", z="
                << p.z << ", t="
                << p.time
                << ")";

            return out;
        }

// class IMU
    // public:
        std::ostream& operator<< (std::ostream& out, const IMU& imu) {
            out << "{" << std::endl
                << "\ta: " << imu.a.transpose() << std::endl
                << "\tw: " << imu.w.transpose() << std::endl
                << "\ttime: " << std::setprecision(16) << imu.time << std::endl
                << "}";
            
            return out;
        }

// class State {
    // public:
        RotTransl State::offsets() {
            return RotTransl(
                this->RLI,
                this->tLI
            );
        }

        void State::operator+= (const IMU& imu) {
            this->update(imu);
        }

        RotTransl operator- (const State& st, const State& s0) {
            Eigen::Matrix3f dR = st.R*s0.R.transpose();
            Eigen::Vector3f dt = st.pos - s0.pos;
            return RotTransl(dR, dt);
        }

        Point State::operator* (const Point& p) {
            return RotTransl(*this) * p;
        }

        RotTransl State::operator* (const RotTransl& RT) {
            return RotTransl(*this) * RT;
        }

    // private:
        // When propagating, we set noises = 0
        void State::propagate_f(IMU imu, float dt) {
            Eigen::Matrix3f Rtemp = this->R;
            Eigen::Vector3f veltemp = this->vel;
            Eigen::Vector3f postemp = this->pos;

            // R ⊞ (w - bw - nw)*dt
            // v ⊞ (R*(a - ba - na) + g)*dt
            // p ⊞ (v*dt + 1/2*(R*(a - ba - na) + g)*dt*dt)

            Rtemp *= SO3::Exp(Eigen::Vector3f(imu.w - this->bw), dt);
            veltemp += (this->R*(imu.a - this->ba) - this->g)*dt;
            postemp += this->vel*dt + 0.5*(this->R*(imu.a - this->ba) - this->g)*dt*dt;

            this->R = Rtemp;
            this->vel = veltemp;
            this->pos = postemp;
        }

        void State::update(IMU imu) {
            // Xt = Xt_1 ⊞ f(imu)*dt
            float dt = imu.time - time;
            this->propagate_f(imu, dt);

            // Update last time
            this->time = imu.time;
        }

// class RotTransl
    // public:
        RotTransl operator* (const RotTransl& RT1, const RotTransl& RT2) {
            return RotTransl(
                RT1.R * RT2.R,
                RT1.R * RT2.t + RT1.t 
            );
        }

        Point operator* (const RotTransl& RT, const Point& p) {
            return Point(
                RT.R*p.toEigen() + RT.t,
                p.time
            );
        }