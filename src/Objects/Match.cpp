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

// class Match {
    // public:
        Match::Match(const Point& p, const Plane& H) {
            this->point = p;
            this->plane = H;
            this->distance = H.dist_to_plane(p);
        }

        bool Match::is_chosen() {
            return this->plane.is_plane;
        }