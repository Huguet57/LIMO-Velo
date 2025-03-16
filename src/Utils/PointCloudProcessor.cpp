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
        Points PointCloudProcessor::downsample(const Points& points) {
            // Downsample points close depending on their time distance
            return this->temporal_downsample(points);
        }

    // private:
        Points PointCloudProcessor::msg2points(const PointCloud_msg& msg) {
            if (Config.LiDAR_type == LIDAR_TYPE::Velodyne) return this->velodynemsg2points(msg);
            if (Config.LiDAR_type == LIDAR_TYPE::Hesai) return this->hesaimsg2points(msg);
            if (Config.LiDAR_type == LIDAR_TYPE::Ouster) return this->oustermsg2points(msg);
            if (Config.LiDAR_type == LIDAR_TYPE::rslidar) return this->rslidarmsg2points(msg);
            
            // Unknown LiDAR type
            RCLCPP_ERROR(rclcpp::get_logger("limovelo"), "Unknown LiDAR type! Change your YAML parameters file.");
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
                if (Config.stamp_beginning) return Conversions::microsec2Sec(pcl.header.stamp) + pcl.points.front().time;
                else return Conversions::microsec2Sec(pcl.header.stamp) + pcl.points.front().time - pcl.points.back().time;
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
                if (Config.stamp_beginning) return Conversions::microsec2Sec(pcl.header.stamp) + Conversions::nanosec2Sec(pcl.points.front().t);
                else return Conversions::microsec2Sec(pcl.header.stamp) + Conversions::nanosec2Sec(pcl.points.front().t) - Conversions::nanosec2Sec(pcl.points.back().t);
            }

        // rslidar specific
            Points PointCloudProcessor::rslidarmsg2points(const PointCloud_msg& msg) {
                pcl::PointCloud<rslidar_ros::Point>::Ptr raw_pcl(new pcl::PointCloud<rslidar_ros::Point>());
                pcl::fromROSMsg(*msg, *raw_pcl);
                return this->to_points(*raw_pcl);
            }

            // Change this to fit the timestamp of the first point
            double PointCloudProcessor::get_begin_time(const pcl::PointCloud<rslidar_ros::Point>& pcl) {
                if (Config.stamp_beginning) return Conversions::microsec2Sec(pcl.header.stamp) + pcl.points.front().timestamp;
                else return Conversions::microsec2Sec(pcl.header.stamp) + pcl.points.front().timestamp - pcl.points.back().timestamp;
            }

        template <typename PointType>
        Points PointCloudProcessor::to_points(const typename pcl::PointCloud<PointType>& pcl) {
            Points pts;

            // Special case: Velodyne points have relative time, we need to get the time stamp from the earliest point
            double begin_time = this->get_begin_time(pcl);

            for (PointType p : pcl.points) pts.push_back(Point (p, begin_time));
            return pts;
        }

        Points PointCloudProcessor::temporal_downsample(const Points& points) {
            Points downsampled;
            int ds_counter = 0;

            for (Point p : points) {
                // Keep point if counter is multiple of downsample_rate
                bool keep_point = Config.downsample_rate <= 1 or ++ds_counter%Config.downsample_rate == 0; 
                if (keep_point and Config.min_dist < p.norm()) downsampled.push_back(p);
            }

            return downsampled;
        }

        bool PointCloudProcessor::time_sort(const Point& a, const Point& b) {
            return a.time < b.time;
        }

        Points PointCloudProcessor::sort_points(const Points& points) {
            Points sorted_points = points;
            // Use std::sort included in <algorithms>
            std::sort(sorted_points.begin(), sorted_points.end(), this->time_sort);
            return sorted_points;
        }

void Processor::fill(pcl::PointCloud<full_info::Point>& pcl, const Points& points) {
    // To then set to max of points
    pcl.header.stamp = 0;

    for (Point p : points) {
        pcl.points.push_back(p.toPCL());
        pcl.header.stamp = std::max(pcl.header.stamp, Conversions::sec2Microsec(p.time));
    }
}