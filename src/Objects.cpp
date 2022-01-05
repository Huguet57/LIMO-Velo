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

// class Buffer
template class Buffer<IMU>;
template class Buffer<Point>;
template class Buffer<State>;

// class Point
    // public:
        Point operator*(const Eigen::Matrix<float, 3, 3>& R, const Point& p) {
            Eigen::Matrix<float, 3, 1> moved_p = R*p.toEigen();
            return Point(moved_p, p);
        }

        Point operator+(const Point& p, const Eigen::Matrix<float, 3, 1> v) {
            Eigen::Matrix<float, 3, 1> moved_p = p.toEigen() + v;
            return Point(moved_p, p);
        }

        Point operator-(const Point& p, const Eigen::Matrix<float, 3, 1> v) {
            return p + (-v);
        }

        std::ostream& operator<< (std::ostream& out, const Point& p) {
            out << "(x="
                << p.x << ", y="
                << p.y << ", z="
                << p.z << ", t="
                << std::setprecision(16)
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
        RotTransl State::I_Rt_L() const {
            return RotTransl(
                this->RLI,
                this->tLI
            );
        }

        RotTransl State::inv() const {
            return RotTransl(*this).inv();
        }

        void State::operator+= (const IMU& imu) {
            this->update(imu);
        }

        Point operator* (const State& X, const Point& p) {
            return RotTransl(X) * p;
        }

        RotTransl operator* (const State& X, const RotTransl& RT) {
            return RotTransl(X) * RT;
        }

        PointCloud operator* (const State& X, const PointCloud& pcl) {
            return RotTransl(X) * pcl;
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

            Rtemp *= SO3Math::Exp(Eigen::Vector3f(imu.w - this->bw), dt);
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

            // Update last controls
            this->time = imu.time;
            this->a = 0.5*this->a + 0.5*imu.a;  // Exponential mean (noisy inputs)
            this->w = 0.5*this->w + 0.5*imu.w;  // Exponential mean (noisy inputs)
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
                p  // attributes
            );
        }

        PointCloud operator* (const RotTransl& RT, const PointCloud& pcl) {
            PointCloud moved_pcl;
            for (PointType p : pcl.points) moved_pcl += RT * Point(p);
            return moved_pcl;
        }

// class Plane
    // public:
        Plane::Plane(const PointVector& points, const std::vector<float>& sq_dists) {
            if (not enough_points(points)) return;
            if (not points_close_enough(sq_dists)) return;
            
            // Given close points, return normal of plane
            this->fit_plane(points);
        }

        float Plane::dist_to_plane(const Point& p) const {
            return n.A * p.x + n.B * p.y + n.C * p.z + n.D;
        }

        template <typename AbstractPoint>
        bool Plane::on_plane(const AbstractPoint& p) {
            return std::fabs(this->dist_to_plane(p)) < Config.PLANES_THRESHOLD;
        }

    // private:
        bool Plane::enough_points(const PointVector& points_near) {
            return this->is_plane = points_near.size() >= Config.NUM_MATCH_POINTS;
        }

        bool Plane::points_close_enough(const std::vector<float>& sq_dists) {
            if (sq_dists.size() < 1) return this->is_plane = false;
            return this->is_plane = sq_dists.back() < Config.MAX_DIST_PLANE*Config.MAX_DIST_PLANE;
        }

        void Plane::fit_plane(const PointVector& points) {
            // Estimate plane
            Eigen::Vector4f ABCD = R3Math::estimate_plane(points);
            this->is_plane = R3Math::is_plane(ABCD, points, Config.PLANES_THRESHOLD);
            
            // Calculate attributes of plane
            if (this->is_plane) {
                this->centroid = R3Math::centroid(points);
                this->n = Normal(ABCD);
            }
        }

// class Normal
    // public:
        Eigen::Matrix<double, 3, 1> operator* (const Eigen::Matrix<double, 3, 3>& R, const Normal& n) {
            return R * n.vect().cast<double> ();
        }

// class Match
    // public:
        bool Match::is_chosen() {
            return this->plane.is_plane;
        }

    // private:
        bool Match::FAST_LIO_HEURISTIC() {
            float s = 1 - 0.9 * std::fabs(this->distance) / std::sqrt(this->point.range);
            return s > 0.9;
        }