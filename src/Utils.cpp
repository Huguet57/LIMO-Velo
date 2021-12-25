#ifndef __OBJECTS_H__
#define __OBJECTS_H__
#include "Common.hpp"
#include "Utils.hpp"
#include "Objects.hpp"
#include "Publishers.hpp"
#include "PointClouds.hpp"
#include "Accumulator.hpp"
#include "Compensator.hpp"
#endif

long long Conversions::sec2Microsec(double t) {
    return std::round(t*1e6);
}

double Conversions::microsec2Sec(long long t) {
    int secs = t/1000000;
    int msecs = t%1000000;
    return secs + msecs*1e-6;
}