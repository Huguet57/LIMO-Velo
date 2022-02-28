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

// class Plane {
    // public:
        Plane::Plane() {}
        Plane::Plane(const PointVector& points, const std::vector<float>& sq_dists) {
            if (not this->enough_points(points)) return;
            if (not this->points_close_enough(sq_dists)) return;
            
            // Given close points, return normal of plane
            this->fit_plane(points);
        }

        float Plane::dist_to_plane(const Point& p) const {
            return n.A * p.x + n.B * p.y + n.C * p.z + n.D;
        }

        bool Plane::on_plane(const Point& p) {
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
