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

// class PointCloudProcessor
    // public:
        PointCloudProcessor::PointCloudProcessor(const PointCloud_msg& msg) {
            PointCloud::Ptr raw_pcl(new PointCloud());
            pcl::fromROSMsg(*msg, *raw_pcl);
            if (raw_pcl->empty()) return;

            Points points = this->to_points(raw_pcl);
            Points downsample_points = this->downsample(points);
            
            this->sort_points(downsample_points);
            this->points = downsample_points;
        }

    // private:
        Points PointCloudProcessor::to_points(const PointCloud::Ptr& pcl) {
            Points pts;
            double begin_time;

            #if LIDAR_TYPE == VELODYNE
                begin_time = Conversions::microsec2Sec(pcl->header.stamp);
                begin_time -= pcl->points.back().time;
            #elif LIDAR_TYPE == HESAI
                begin_time = 0.;
            #else
                ROS_ERROR("INVALID LiDAR TYPE");
            #endif

            for (PointType p : pcl->points) pts.push_back(Point (p, begin_time));
            return pts;
        }

        Points PointCloudProcessor::downsample(const Points& points) {
            Points downsampled;
            int ds_counter = 0;

            for (Point p : points) {
                bool downsample_point = Config.ds_rate <= 1 or ++ds_counter%Config.ds_rate == 0; 
                if (downsample_point and Config.min_dist < p.norm()) downsampled.push_back(p);
            }

            return downsampled;
        }

        bool PointCloudProcessor::time_sort(const Point& a, const Point& b) {
            return a.time < b.time;
        }

        void PointCloudProcessor::sort_points(Points& points) {
            std::sort(points.begin(), points.end(), this->time_sort);
        }

void operator+= (PointCloud& pcl, const Point& p) {
    pcl.points.push_back(p.toPCL());
}
