#ifndef SLAM_LIBRARIES_H
#define SLAM_LIBRARIES_H
// SLAM libraries
#include "use-ikfom.hpp"
#include "ikd_Tree.h"
#endif

class Localizator {
    public:
        Mapper* map;
        PointCloud points2match;
    private:
        esekfom::esekf<state_ikfom, 12, input_ikfom> KF;
    
    public:
        Localizator();
        Localizator(Mapper*);
        void update(const PointCloud&);
        void calculate_H(const state_ikfom&, const Planes&, Eigen::MatrixXd& H, Eigen::VectorXd& h);
    
    private:
        void init_IKFoM();
        void IKFoM_update(double&);

    // Singleton pattern
    public:
        static Localizator& getInstance();
    
    private:
        // Localizator() = default;

        // Delete copy/move so extra instances can't be created/moved.
        Localizator(const Localizator&) = delete;
        Localizator& operator=(const Localizator&) = delete;
        Localizator(Localizator&&) = delete;
        Localizator& operator=(Localizator&&) = delete;

};