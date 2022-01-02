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

        void add(PointCloud&, double time, bool downsample=false);        
        void add(const State&, PointCloud&, bool downsample=false);        
        Planes match(const State&, const PointCloud&);
        bool hasToMap(double t);

    private:
        void init_tree();
        void build_tree(PointCloud&);
        bool exists_tree();

        void add_points(PointCloud&, bool downsample=false);
        Plane match_plane(const State&, const PointType&);

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