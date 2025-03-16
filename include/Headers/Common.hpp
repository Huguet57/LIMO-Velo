#ifndef COMMON_H
#define COMMON_H
// Libraries
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <math.h>
#include <chrono>
// Data Structures
#include <deque>
#include <vector>
// TF library
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// ROS messages
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "common_msgs/msg/state.hpp"
// PCL Library
#define PCL_NO_PRECOMPILE
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#ifndef SLAM_LIBRARIES_H
#define SLAM_LIBRARIES_H
// SLAM libraries
#include "use-ikfom.hpp"
#include "ikd_Tree.h"
#endif

namespace LIDAR_TYPE {
    const std::string Velodyne = "velodyne";
    const std::string Hesai = "hesai";
    const std::string Ouster = "ouster";
    const std::string rslidar = "rslidar";
}

struct InitializationParams {
    std::vector<double> times;
    std::vector<double> deltas;
};

struct Params {
    bool mapping;
    bool real_time;
    bool pub_full_pcl;

    bool estimate_extrinsics;
    bool print_extrinsics;
    std::vector<double> initial_gravity;
    std::vector<double> I_Rotation_L;
    std::vector<double> I_Translation_L;

    double time_without_clear_BUFFER_L;
    double time_without_clear_buffer_mapped_points;
    double real_time_delay;

    double full_rotation_time;
    double imu_rate;

    int downsample_rate;
    float downsample_prec;
    bool high_quality_publish;

    double min_dist;
    std::string LiDAR_type;
    
    bool offset_beginning;
    bool stamp_beginning;

    double degeneracy_threshold;
    bool print_degeneracy_values;
    bool print_debug;

    int MAX_NUM_ITERS;
    int MAX_POINTS2MATCH;
    std::vector<double> LIMITS;
    int NUM_MATCH_POINTS;
    double MAX_DIST_PLANE;
    float PLANES_THRESHOLD;
    float PLANES_CHOOSE_CONSTANT;

    double wx_MULTIPLIER;
    double wy_MULTIPLIER;
    double wz_MULTIPLIER;

    double cov_acc;
    double cov_gyro;
    double cov_bias_acc;
    double cov_bias_gyro;
    double LiDAR_noise;

    std::string points_topic;
    std::string imus_topic;

    InitializationParams Initialization;
};

namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

namespace hesai_ros {
    struct Point {
        PCL_ADD_POINT4D
        uint8_t intensity;
        double timestamp;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
}

namespace rslidar_ros {
    struct Point {
        PCL_ADD_POINT4D
        float intensity;
        double timestamp;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
}


namespace full_info {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      PCL_ADD_RGB;
      float intensity;
      float range;
      double timestamp;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

namespace custom {
    // Example: point with lots of fields
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        PCL_ADD_RGB;
        float intensity;
        float range;
        double timestamp;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      std::uint32_t t;
      std::uint16_t reflectivity;
      std::uint8_t  ring;
      std::uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint8_t, intensity, intensity)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)


POINT_CLOUD_REGISTER_POINT_STRUCT(full_info::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, r, r)
    (float, g, g)
    (float, b, b)
    (float, intensity, intensity)
    (float, range, range)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint32_t, range, range)
)

// Example: point with lots of fields
POINT_CLOUD_REGISTER_POINT_STRUCT(custom::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, r, r)
    (float, g, g)
    (float, b, b)
    (float, intensity, intensity)
    (float, range, range)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)

typedef sensor_msgs::msg::PointCloud2::SharedPtr PointCloud_msg;
typedef common_msgs::msg::State State_msg;
typedef double TimeType;

class Point;
class IMU;
class State;
class RotTransl;
typedef std::deque<Point> Points;
typedef std::vector<Point, Eigen::aligned_allocator<Point>> PointVector;
typedef std::deque<IMU> IMUs;
typedef std::deque<State> States;

class Normal;
class Plane;
class Match;
typedef std::vector<Normal> Normals;
typedef std::vector<Plane> Planes;
typedef std::vector<Match> Matches;

class Accumulator;
class Localizator;
class Mapper;

#endif