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

// class Mapper
    // public:
        Mapper::Mapper() {  // TODO: (const KDTREE_OPTIONS& options) {
            this->init_tree();
        }

        void Mapper::add(PointCloud& pcl, double time, bool downsample) {
            if (pcl.empty()) return;

            // If map doesn't exist, build it.
            if (not this->exists()) this->build_tree(pcl);
            else this->add_points(pcl, downsample);

            this->last_map_time = time;
        }

        int Mapper::size() {
            return this->map->size(); 
        }

        bool Mapper::exists() {
            return this->exists_tree();
        }

        Matches Mapper::match(const State& X, const PointCloud& points) {            
            Matches matches;
            if (not this->exists()) return matches;
            matches.reserve(points.size());
            
            omp_set_num_threads(MP_PROC_NUM);
            #pragma omp parallel for
            for (PointType p : points) {
                // Direct approach: we match the point with a plane on the map
                Match match = this->match_plane(X * X.I_Rt_L() * Point(p));
                if (match.is_chosen()) matches.push_back(match);
            }

            return matches;
        }

        bool Mapper::hasToMap(double t) {
            if (this->last_map_time < 0) this->last_map_time = t;
            return t - this->last_map_time >= Config.full_rotation_time;
        }

    // private:
        void Mapper::init_tree() {  // TODO: (const KDTREE_OPTIONS& options) {
            this->map = KD_TREE<Point>::Ptr (new KD_TREE<Point>(0.3, 0.6, 0.2));
        }

        void Mapper::build_tree(PointCloud& pcl) {
            PointVector pts; for (PointType p : pcl.points) pts.push_back(Point(p));
            this->map->Build(pts);
        }

        void Mapper::add_points(PointCloud& pcl, bool downsample) {
            PointVector pts; for (PointType p : pcl.points) pts.push_back(Point(p));
            this->map->Add_Points(pts, downsample);
        }

        bool Mapper::exists_tree() {
            return this->map->size() > 0;
        }

        Match Mapper::match_plane(const Point& p) {
            // Find k nearest points
            PointVector near_points;
            vector<float> pointSearchSqDis(Config.NUM_MATCH_POINTS);
            this->map->Nearest_Search(p, Config.NUM_MATCH_POINTS, near_points, pointSearchSqDis);

            // Construct a plane fitting between them
            return Match(p, Plane (near_points, pointSearchSqDis));
        }