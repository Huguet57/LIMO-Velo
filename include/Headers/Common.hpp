#ifndef COMMON_H
#define COMMON_H
// Libraries
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <chrono>
// Data Structures
#include <deque>
#include <vector>
// TF library
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
// ROS messages
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
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
    const int Velodyne = 0;
    const int Hesai = 1;
    const int Custom = 2;
}

struct HeuristicParams {
    std::vector<double> times;
    std::vector<double> deltas;
};

struct Params {
    bool mapping_online;
    bool real_time;

    bool estimate_extrinsics;
    bool print_extrinsics;
    std::vector<float> initial_gravity;
    std::vector<float> I_Rotation_L;
    std::vector<float> I_Translation_L;

    double empty_lidar_time;
    double real_time_delay;

    double full_rotation_time;
    double imu_rate;

    int ds_rate;
    double min_dist;
    int LiDAR_type;
    
    double degeneracy_threshold;
    bool print_degeneracy_values;

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

    HeuristicParams Heuristic;
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

typedef sensor_msgs::PointCloud2::ConstPtr PointCloud_msg;
typedef sensor_msgs::ImuConstPtr IMU_msg;
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

class Localizator;
class Mapper;

#endif