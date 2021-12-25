// Libraries
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <chrono>
// Data Structures
#include <deque>
#include <vector>
// ROS messages
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
// PCL Library
#define PCL_NO_PRECOMPILE
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

struct Params {
    double delta;
    int rate;
    
    double empty_lidar_time;
    double real_time_delay;
    
    int ds_rate;
    double min_dist;
    
    std::string points_topic;
    std::string imus_topic;
};

namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring)
)

typedef double TimeType;
typedef velodyne_ros::Point PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef sensor_msgs::PointCloud2::ConstPtr PointCloud_msg;
typedef sensor_msgs::ImuConstPtr IMU_msg;

class Point;
class IMU;
class State;
typedef std::deque<Point> Points;
typedef std::deque<IMU> IMUs;
typedef std::deque<State> States;

class RotTransl;