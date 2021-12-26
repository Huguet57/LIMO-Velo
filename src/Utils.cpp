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

long long Conversions::sec2Microsec(double t) {
    return std::round(t*1e6);
}

double Conversions::microsec2Sec(long long t) {
    int secs = t/1000000;
    int msecs = t%1000000;
    return secs + msecs*1e-6;
}