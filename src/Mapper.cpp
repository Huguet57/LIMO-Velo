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

        void Mapper::add(PointCloud& pcl, bool downsample) {
            // If map doesn't exist, build it.
            if (pcl.size() == 0) return;
            if (not this->exists()) this->build_tree(pcl);
            else this->add_points(pcl, downsample);
        }

        int Mapper::size() {
            return this->map->size(); 
        }

        bool Mapper::exists() {
            return this->exists_tree();
        }

        Planes Mapper::match(const State& X, const PointCloud& points) {            
            Planes matches;
            if (not this->exists()) return matches;
            matches.reserve(points.size());

            for (PointType p : points) {
                // Direct approach: we match the point with a plane on the map
                Plane matched_plane = this->match_plane(X, p);
                if (matched_plane.is_plane) matches.push_back(matched_plane);
            }

            return matches;
        }

        bool Mapper::hasToMap(double t) {
            if (this->last_map_time < 0) this->last_map_time = t;
            else return t - this->last_map_time >= Config.full_rotation_time;
            return false;
        }

    // private:
        void Mapper::init_tree() {  // TODO: (const KDTREE_OPTIONS& options) {
            this->map = KD_TREE<PointType>::Ptr (new KD_TREE<PointType>(0.3, 0.6, 0.2));
        }

        void Mapper::build_tree(PointCloud& pcl) {
            this->map->Build(pcl.points);
        }

        void Mapper::add_points(PointCloud& pcl, bool downsample) {
            this->map->Add_Points(pcl.points, downsample);
        }

        bool Mapper::exists_tree() {
            return this->map->size() > 0;
        }

        Plane Mapper::match_plane(const State& X, const PointType& p) {
            // Transport the point to the global frame
            PointTypes near_points;
            RotTransl offsets = X.I_Rt_L();
            PointType global_p = (X * offsets * Point(p)).toPCL();

            // Find k nearest points
            const int NUM_MATCH_POINTS = 5;
            vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
            this->map->Nearest_Search(global_p, NUM_MATCH_POINTS, near_points, pointSearchSqDis);

            // Fit a plane to them
            return Plane(p, near_points, pointSearchSqDis);
        }