#ifndef SLAM_LIBRARIES_H
#define SLAM_LIBRARIES_H
// SLAM libraries
#include "use-ikfom.hpp"
#include "ikd_Tree.h"
#endif

class Mapper {
    public:
        double last_map_time = -1;
        bool frozen = false;

    private:
        KD_TREE<Point>::Ptr map;

    public:
        Mapper();
        bool exists();
        int size();

        void add(Points&, double time, bool downsample=false);        
        void add(const State&, Points&, bool downsample=false);        
        Matches match(const State&, const Points&);
        bool hasToMap(double t);

        Points radius_search(const Point& center, double radius);

        bool load(const std::filesystem::path&);
        bool save(const std::filesystem::path&);

        void freeze();

    private:
        void init_tree();
        void build_tree(Points&);
        bool exists_tree();

        void add_points(Points&, bool downsample=false);
        Match match_plane(const Point&);

        Points tree_radius_search(const Point& center, double radius);

        bool valid_file(const std::filesystem::path&);
        bool valid_path(const std::filesystem::path&);

        Points extract_points(const KD_TREE<Point>::Ptr& tree);
        Points extract_points(const std::filesystem::path&);
        void create_map_file(const std::filesystem::path&, const Points&);

    // Singleton pattern
    public:
        static Mapper& getInstance() {
            static Mapper* mapper = new Mapper();
            return *mapper;
        }

    private:
        // Mapper() = default;

        // Delete copy/move so extra instances can't be created/moved.
        Mapper(const Mapper&) = delete;
        Mapper& operator=(const Mapper&) = delete;
        Mapper(Mapper&&) = delete;
        Mapper& operator=(Mapper&&) = delete;
};