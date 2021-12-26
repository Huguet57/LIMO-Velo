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
        Localizator(Mapper&);
        void update(const PointCloud&);
        void h_share_model(state_ikfom&, esekfom::dyn_share_datastruct<double>&);
    private:
        void init_IKFoM();
        void calculate_H(const state_ikfom&, const Planes&, Eigen::MatrixXd& H, Eigen::VectorXd& h);
        void IKFoM_update(double&);
};