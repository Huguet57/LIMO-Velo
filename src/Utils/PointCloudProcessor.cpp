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
            Points points = this->msg2points(msg);
            Points downsample_points = this->downsample(points);
            
            this->sort_points(downsample_points);
            this->points = downsample_points;
        }

    // private:
        Points PointCloudProcessor::msg2points(const PointCloud_msg& msg) {
            if (Config.LiDAR_type == LIDAR_TYPE::Velodyne) return this->velodynemsg2points(msg);
            if (Config.LiDAR_type == LIDAR_TYPE::Hesai) return this->hesaimsg2points(msg);
            if (Config.LiDAR_type == LIDAR_TYPE::Ouster) return this->oustermsg2points(msg);
            if (Config.LiDAR_type == LIDAR_TYPE::Custom) return this->custommsg2points(msg);
            
            // Unknown LiDAR type
            ROS_ERROR("Unknown LiDAR type! Change your YAML parameters file.");
            return Points ();
        }

        // Velodyne specific
            Points PointCloudProcessor::velodynemsg2points(const PointCloud_msg& msg) {
                pcl::PointCloud<velodyne_ros::Point>::Ptr raw_pcl(new pcl::PointCloud<velodyne_ros::Point>());
                pcl::fromROSMsg(*msg, *raw_pcl);
                return this->to_points(*raw_pcl);
            }

            double PointCloudProcessor::get_begin_time(const pcl::PointCloud<velodyne_ros::Point>& pcl) {
                // Velodyne points have relative time
                return Conversions::microsec2Sec(pcl.header.stamp) - pcl.points.back().time + pcl.points.front().time;
            }

        // HESAI specific
            Points PointCloudProcessor::hesaimsg2points(const PointCloud_msg& msg) {
                pcl::PointCloud<hesai_ros::Point>::Ptr raw_pcl(new pcl::PointCloud<hesai_ros::Point>());
                pcl::fromROSMsg(*msg, *raw_pcl);
                return this->to_points(*raw_pcl);
            }

            double PointCloudProcessor::get_begin_time(const pcl::PointCloud<hesai_ros::Point>& pcl) {
                // HESAI points have absolute time
                return 0.d;
            }

        // Ouster specific
            Points PointCloudProcessor::oustermsg2points(const PointCloud_msg& msg) {
                pcl::PointCloud<ouster_ros::Point>::Ptr raw_pcl(new pcl::PointCloud<ouster_ros::Point>());
                pcl::fromROSMsg(*msg, *raw_pcl);
                return this->to_points(*raw_pcl);
            }

            double PointCloudProcessor::get_begin_time(const pcl::PointCloud<ouster_ros::Point>& pcl) {
                // Ouster points have relative time
                return Conversions::microsec2Sec(pcl.header.stamp) - Conversions::microsec2Sec(pcl.points.back().t)*1e-3 + Conversions::microsec2Sec(pcl.points.front().t)*1e-3;
            }

        // Custom specific
            Points PointCloudProcessor::custommsg2points(const PointCloud_msg& msg) {
                pcl::PointCloud<custom::Point>::Ptr raw_pcl(new pcl::PointCloud<custom::Point>());
                pcl::fromROSMsg(*msg, *raw_pcl);
                return this->to_points(*raw_pcl);
            }

            // Change this to fit the timestamp of the first point
            double PointCloudProcessor::get_begin_time(const pcl::PointCloud<custom::Point>& pcl) {
                // // Example: Points with relative time
                // return Conversions::microsec2Sec(pcl.header.stamp) - pcl.points.back().time + pcl.points.front().time;
                
                // Example: Points with absolute time
                return 0.d;
            }

        template <typename PointType>
        Points PointCloudProcessor::to_points(const typename pcl::PointCloud<PointType>& pcl) {
            Points pts;

            // Special case: Velodyne points have relative time, we need to get the time stamp from the earliest point
            double begin_time = this->get_begin_time(pcl);

            for (PointType p : pcl.points) pts.push_back(Point (p, begin_time));
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

void Processor::fill(pcl::PointCloud<full_info::Point>& pcl, const Points& points) {
    // To then set to max of points
    pcl.header.stamp = 0;

    for (Point p : points) {
        pcl.points.push_back(p.toPCL());
        pcl.header.stamp = std::max(pcl.header.stamp, Conversions::sec2Microsec(p.time));
    }
}