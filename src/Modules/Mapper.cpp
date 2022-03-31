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

        void Mapper::add(Points& points, double time, bool downsample) {
            if (points.empty()) return;
            if (this->frozen) return;

            // If map doesn't exist, build it.
            if (not this->exists()) this->build_tree(points);
            else this->add_points(points, downsample);

            this->last_map_time = time;
        }

        int Mapper::size() {
            return this->map->size(); 
        }

        bool Mapper::exists() {
            return this->exists_tree();
        }

        Matches Mapper::match(const State& X, const Points& points) {            
            Matches matches;
            if (not this->exists()) return matches;
            matches.reserve(points.size());
            
            omp_set_num_threads(MP_PROC_NUM);
            #pragma omp parallel for
            for (int pi = 0; pi < points.size(); ++pi) {
                Point p = points[pi];

                // Direct approach: we match the point with a plane on the map
                Match match = this->match_plane(X * X.I_Rt_L() * p);
                if (match.is_chosen()) matches.push_back(match);
            }

            return matches;
        }

        Points Mapper::radius_search(const Point& center, double radius) {
            return this->tree_radius_search(center, radius);
        }

        bool Mapper::hasToMap(double t) {
            if (this->last_map_time < 0) this->last_map_time = t;
            return t - this->last_map_time >= Config.full_rotation_time;
        }


        bool Mapper::load(const std::filesystem::path& fullpath) {
            if (not this->valid_file(fullpath)) return false;

            // Load points on the tree
            Points loaded_points = this->extract_points(fullpath);
            this->build_tree(loaded_points);
            this->freeze();

            return true;
        }

        bool Mapper::save(const std::filesystem::path& fullpath) {
            if (not this->valid_path(fullpath)) {
                ROS_ERROR("Maps' path doesn't exists or is not valid. Change the YAML 'maps_path' parameter.");
                return false;
            }

            // Extract points from tree and save to file
            Points extracted_points = this->extract_points(this->map);
            this->create_map_file(fullpath, extracted_points);

            // Success!
            ROS_INFO("Map saved successfully at %s", fullpath.c_str());
            return true;
        }

        void Mapper::freeze() {
            this->frozen = true;
        }

    // private:
        void Mapper::init_tree() {  // TODO: (const KDTREE_OPTIONS& options) {
            this->map = KD_TREE<Point>::Ptr (new KD_TREE<Point>(0.3, 0.6, 0.2));
        }

        void Mapper::build_tree(Points& points) {
            PointVector as_vector; for (Point p : points) as_vector.push_back(p);
            this->map->Build(as_vector);
        }

        void Mapper::add_points(Points& points, bool downsample) {
            PointVector as_vector; for (Point p : points) as_vector.push_back(p);
            this->map->Add_Points(as_vector, downsample);
        }

        bool Mapper::exists_tree() {
            return this->map->size() > 0;
        }

        Points Mapper::tree_radius_search(const Point& center, double radius) {
            PointVector point_vect;
            this->map->Radius_Search(center, radius, point_vect);

            Points points;
            for (Point p : point_vect) points.push_back(p);
            return points;
        }

        Match Mapper::match_plane(const Point& p) {
            // Find k nearest points
            PointVector near_points;
            vector<float> pointSearchSqDis(Config.NUM_MATCH_POINTS);
            this->map->Nearest_Search(p, Config.NUM_MATCH_POINTS, near_points, pointSearchSqDis);

            // Construct a plane fitting between them
            return Match(p, Plane (near_points, pointSearchSqDis));
        }

        bool Mapper::valid_file(const std::filesystem::path& path) {
            // C++17 <filesystem> way to check if a directory exists
            return std::filesystem::exists(path);
        }

        bool Mapper::valid_path(const std::filesystem::path& path) {
            // C++17 <filesystem> way to check if a directory exists
            return std::filesystem::exists(path.parent_path()) and std::filesystem::is_directory(path.parent_path());
        }

        Points Mapper::extract_points(const KD_TREE<Point>::Ptr& tree) {
            PointVector point_vect;
            // ikd-Tree's flatten recursively adds all points in tree to 'points'
            tree->flatten(tree->Root_Node, point_vect, NOT_RECORD);

            Points points;
            for (Point p : point_vect) points.push_back(p);
            return points;
        }

        Points Mapper::extract_points(const std::filesystem::path& path) {
            Points points;
            std::ifstream map_file(path.u8string());

            // Read line-by-line
            std::string line;
            while (std::getline(map_file, line)) {
                // Each line must be a point
                Point p;
                std::istringstream iss(line);

                // If no problems, add to points
                if (iss >> p.x >> p.y >> p.z >> p.intensity >> p.range >> p.time)
                    points.push_back(p);
            }

            return points;
        }

        void Mapper::create_map_file(const std::filesystem::path& fullpath, const Points& points) {
            // Create a map file in given path
            std::ofstream map_file;
            map_file.open(fullpath);

            // Add Point information line-by-line
            for (Point p : points)
                map_file
                    // XYZ
                    << p.x << " " << p.y << " " << p.z
                    // Attributes
                    << " " << p.intensity << " " << p.range
                    // Time
                    << " " << p.time
                    // Line break
                    << std::endl;

            map_file.close();
        }