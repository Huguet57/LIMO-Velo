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

uint64_t Conversions::sec2Microsec(double t) {
    return std::round(t*1e6);
}

double Conversions::microsec2Sec(uint64_t t) {
    int secs = t/1000000;
    int msecs = t%1000000;
    return secs + msecs*1e-6;
}

Eigen::Matrix<float, 4, 1> R3Math::estimate_plane(const PointVector &point) {
    int NUM_MATCH_POINTS = point.size();
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A(NUM_MATCH_POINTS, 3);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> b(NUM_MATCH_POINTS, 1);
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }

    Eigen::Matrix<float, 3, 1> normvec = A.colPivHouseholderQr().solve(b);
    Eigen::Matrix<float, 4, 1> pca_result;

    float n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    return pca_result;
}

bool R3Math::is_plane(const Eigen::Matrix<float, 4, 1> &pca_result, const PointVector &point, const float &threshold) {
    for (int j = 0; j < point.size(); j++) {
        float res = pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3);
        if (fabs(res) > threshold) return false;
    }

    return true;
}

Point R3Math::centroid(const PointVector& pts) {
    int N = pts.size();
    Eigen::Matrix<float, 3, 1> centroid_vect;
    for (Point p : pts) centroid_vect += p.toEigen();
    return Point (centroid_vect/N);
}