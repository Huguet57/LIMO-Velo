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

// class Normal {
    // public:

        Normal::Normal() {}
        
        Normal::Normal(const Eigen::Matrix<float, 4, 1>& ABCD) {
            this->A = ABCD(0);
            this->B = ABCD(1);
            this->C = ABCD(2);
            this->D = ABCD(3);
        }

        Eigen::Matrix<float, 3, 1> Normal::vect() const {
            return Eigen::Matrix<float, 3, 1> (
                this->A,
                this->B,
                this->C
            );
        }

        Eigen::Matrix<double, 3, 1> operator* (const Eigen::Matrix<double, 3, 3>& R, const Normal& n) {
            return R * n.vect().cast<double> ();
        }