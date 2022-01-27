#ifndef SLAM_LIBRARIES_H
#define SLAM_LIBRARIES_H
// SLAM libraries
#include "use-ikfom.hpp"
#include "ikd_Tree.h"
#endif

class Mapper {
    public:
        double last_map_time = -1;

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

    private:
        void init_tree();
        void build_tree(Points&);
        bool exists_tree();

        void add_points(Points&, bool downsample=false);
        Match match_plane(const Point&);

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