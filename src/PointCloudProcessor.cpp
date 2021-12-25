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

extern struct Params Config;

// class PointCloudProcessor
    // public:
        PointCloudProcessor::PointCloudProcessor(const PointCloud_msg& msg, Buffer<Point>& BUFFER_L) {
            PointCloud::Ptr raw_pcl(new PointCloud());
            pcl::fromROSMsg(*msg, *raw_pcl);
            
            PointCloud downsample_points = this->downsample(raw_pcl);
            this->sort_points(downsample_points.points);
            this->add2Buffer(downsample_points, BUFFER_L);
        }

    // private:
        template <typename AbstractPoint>
        double dist(const AbstractPoint& p) {
            return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        }

        PointCloud PointCloudProcessor::downsample(const PointCloud::Ptr& pcl) {
            PointCloud::Ptr downsampled(new PointCloud());
            downsampled->points.reserve(pcl->points.size()/Config.ds_rate);
            downsampled->header = pcl->header;
            int ds_counter = 0;
            for (PointType p : pcl->points)
                if (++ds_counter%Config.ds_rate == 0 and Config.min_dist < dist(p))
                    downsampled->points.push_back(p);

            return *downsampled;

            // PointCloud::Ptr filtered(new PointCloud());
            // filtered->header = downsampled->header;
            // pcl::VoxelGrid<PointType> filter;
            // filter.setInputCloud (downsampled);
            // filter.setLeafSize (0.05f, 0.05f, 0.05f);
            // filter.filter (*filtered);
            // return *filtered;
        }

        void PointCloudProcessor::add2Buffer(const PointCloud& pcl, Buffer<Point>& BUFFER_L) {
            double begin_time = Conversions::microsec2Sec(pcl.header.stamp) - pcl.points.back().time; 
            for (PointType p : pcl.points) BUFFER_L.push(Point(p, begin_time));
        }

        bool PointCloudProcessor::time_sort(const PointType& a, const PointType& b) {
            return a.time < b.time;
        }

        void PointCloudProcessor::sort_points(std::vector<PointType, Eigen::aligned_allocator<PointType>>& points) {
            std::sort(points.begin(), points.end(), this->time_sort);
        }

void operator+= (PointCloud& pcl, const Point& p) {
    pcl.points.push_back(p.toPCL());
}