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

// class Buffer
template class Buffer<IMU>;
template class Buffer<Point>;
template class Buffer<State>;

// class Point
    // public:
        Point operator*(const Eigen::Matrix<float, 3, 3>& R, const Point& p) {
            Eigen::Matrix<float, 3, 1> moved_p = R*p.toEigen();
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
        RotTransl State::I_Rt_L() const {
            return RotTransl(
                this->RLI,
                this->tLI
            );
        }

        void State::operator+= (const IMU& imu) {
            this->update(imu);
        }

        RotTransl operator- (const State& st, const State& s0) {
            Eigen::Matrix3f dR = s0.R.transpose()*st.R;
            Eigen::Vector3f dt = st.pos - s0.pos;
            return RotTransl(dR, dt);
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
            this->a = imu.a;
            this->w = imu.w;
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

        PointCloud operator* (const RotTransl& RT, const PointCloud& pcl) {
            PointCloud moved_pcl;
            for (PointType p : pcl.points) moved_pcl += RT * Point(p);
            return moved_pcl;
        }

// class Plane
    // public:
        Plane::Plane(const PointType& p, const PointTypes& points, const std::vector<float>& sq_dists) {
            if (not enough_points(points)) return;
            if (not points_close_enough(sq_dists)) return;
            
            // Given close points, return normal of plane
            this->calculate_attributes(p, points);
        }

        template <typename AbstractPoint>
        bool Plane::on_plane(const AbstractPoint& p, float& res) {
            // Calculate residue
            res = n.A * p.x + n.B * p.y + n.C * p.z + n.D;
            return std::fabs(res < 0.1f);

            // float s = 1 - 0.9 * fabs(res) / sqrt(dist); // TODO: Why divide by dist?
            // return s > 0.9;
        }

    // private:
        bool Plane::enough_points(const PointTypes& points_near) {
            return this->is_plane = points_near.size() >= this->NUM_MATCH_POINTS;
        }

        bool Plane::points_close_enough(const std::vector<float>& sq_dists) {
            if (sq_dists.size() < 1) return this->is_plane = false;
            return this->is_plane = sq_dists.back() < MAX_DIST*MAX_DIST;
        }

        void Plane::calculate_attributes(const PointType& p, const PointTypes& points) {
            Eigen::Vector4f normal_ampl;
            if (this->is_plane = this->estimate_plane(normal_ampl, points, 0.1f)) {
                // Centroid
                this->centroid.x = p.x;
                this->centroid.y = p.y;
                this->centroid.z = p.z;

                // Normal
                this->n.A = normal_ampl(0);
                this->n.B = normal_ampl(1);
                this->n.C = normal_ampl(2);
                this->n.D = normal_ampl(3);

                // Distance
                this->on_plane(this->centroid, this->distance);
            }
        }

        template<typename T>
        bool Plane::estimate_plane(Eigen::Matrix<T, 4, 1> &pca_result, const PointTypes &point, const T &threshold)
        {
            int N = this->NUM_MATCH_POINTS;
            Eigen::Matrix<T, 5, 3> A;
            Eigen::Matrix<T, 5, 1> b;
            A.setZero();
            b.setOnes();
            b *= -1.0f;

            for (int j = 0; j < N; j++)
            {
                A(j,0) = point[j].x;
                A(j,1) = point[j].y;
                A(j,2) = point[j].z;
            }

            Eigen::Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

            T n = normvec.norm();
            pca_result(0) = normvec(0) / n;
            pca_result(1) = normvec(1) / n;
            pca_result(2) = normvec(2) / n;
            pca_result(3) = 1.0 / n;

            for (int j = 0; j < N; j++)
            {
                if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
                {
                    return false;
                }
            }
            return true;
        }

// struct Normal
    Eigen::Matrix<double, 3, 1> operator* (const Eigen::Matrix<double, 3, 3>& R, const Normal& n) {
        double A = (double) n.A;
        double B = (double) n.B;
        double C = (double) n.C;
        return R * Eigen::Vector3d(A, B, C);
    }