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

// class Point {
    // public:

        Point::Point() {}

        Point::Point(const full_info::Point& p) {
            this->set_XYZ(p);
            this->set_attributes(p);
            this->time = p.timestamp;
        }
  
        Point::Point(const Eigen::Matrix<float, 3, 1>& p) {
            this->set_XYZ(p);
        }

        // Delegate constructor (Eigen + attributes)
        Point::Point(const Eigen::Matrix<float, 3, 1>& p, const Point& attributes) : Point::Point(p) {
            this->pass_attributes(attributes);
            this->time = attributes.time;
        }

        // HESAI specific
            Point::Point(const hesai_ros::Point& p) {
                this->set_XYZ(p);
                this->set_attributes(p);
                this->time = p.timestamp;
            }

            Point::Point(const hesai_ros::Point& p, double time_offset)
                // Construct point as usual first
                : Point::Point (p)
            {
                // Then add the time offset to the Point's time
                this->time += time_offset;
            }
        
        // Velodyne specific
            Point::Point(const velodyne_ros::Point& p) {
                this->set_XYZ(p);
                this->set_attributes(p);
                
                // Time offset with respect to beginning of rotation, i.e. ~= [0, 0.1]
                if (Config.offset_beginning) this->time = (double) p.time;
                // Time offset with respect to end of rotation, i.e. ~= [-0.1, 0]
                else this->time = Config.full_rotation_time + (double) p.time;
            }

            Point::Point(const velodyne_ros::Point& p, double time_offset)
                // Construct point as usual first
                : Point::Point (p)
            {
                // Then add the time offset to the Point's time
                this->time += time_offset;
            }

        // Ouster specific
            Point::Point(const ouster_ros::Point& p) {
                this->set_XYZ(p);
                this->set_attributes(p);

                // Time offset with respect to beginning of rotation, i.e. ~= [0, 0.1]
                if (Config.offset_beginning) this->time = Conversions::nanosec2Sec(p.t);
                // Time offset with respect to end of rotation, i.e. ~= [-0.1, 0]
                else this->time = Config.full_rotation_time + Conversions::nanosec2Sec(p.t);
            }

            Point::Point(const ouster_ros::Point& p, double time_offset)
                // Construct point as usual first
                : Point::Point (p)
            {
                // Then add the time offset to the Point's time
                this->time += time_offset;
            }

        // rslidar specific
            Point::Point(const rslidar_ros::Point& p) {
                this->set_XYZ(p);

                // -------------------------------------------
                //      View below to modify this method
                // -------------------------------------------
                this->set_attributes(p);

                // -------------------------------------------
                //      Choose the appropiate time field
                // -------------------------------------------
                this->time = p.timestamp; // or p.time
            }

            Point::Point(const rslidar_ros::Point& p, double time_offset)
                // Construct point as usual first
                : Point::Point (p)
            {
                // Then add the time offset to the Point's time
                this->time += time_offset;
            }

        full_info::Point Point::toPCL() const {
            full_info::Point p;
            p.x = this->x;
            p.y = this->y;
            p.z = this->z;
            p.timestamp = this->time;
            p.intensity = this->intensity;
            p.range = this->range;
            return p;
        }

        Eigen::Matrix<float, 3, 1> Point::toEigen() const {
            return Eigen::Matrix<float, 3, 1>(this->x, this->y, this->z);
        }

        float Point::norm() const {
            return this->toEigen().norm(); 
        }

        Eigen::Vector3d Point::cross(const Eigen::Vector3d& v) {
            Eigen::Vector3d w = this->toEigen().cast<double> ();
            return w.cross(v);
        }

        Point operator*(const Eigen::Matrix<float, 3, 3>& R, const Point& p) {
            Eigen::Matrix<float, 3, 1> moved_p = R*p.toEigen();
            return Point(moved_p, p);
        }

        Point operator+(const Point& p, const Eigen::Matrix<float, 3, 1> v) {
            Eigen::Matrix<float, 3, 1> moved_p = p.toEigen() + v;
            return Point(moved_p, p);
        }

        Point operator-(const Point& p, const Eigen::Matrix<float, 3, 1> v) {
            return p + (-v);
        }

    // private:
        template <typename PointType>
        void Point::set_XYZ(const PointType& p) {
            this->x = p.x;
            this->y = p.y;
            this->z = p.z;
        }

        void Point::set_XYZ(const Eigen::Matrix<float, 3, 1>& p) {
            this->x = p(0);
            this->y = p(1);
            this->z = p(2);
        }

        template <typename PointType>
        void Point::set_attributes(const PointType& p) {
            this->intensity = p.intensity;
            this->range = this->norm();
        }
        
        // Ouster specific
        void Point::set_attributes(const ouster_ros::Point& p) {
            this->intensity = p.reflectivity;
            this->range = p.range;
        }

        // ---------------------------------------------------------------------------------------
        //      Uncomment and modify this if point type doesn't have 'intensity' as attribute
        // ---------------------------------------------------------------------------------------
        
        // Point::set_attributes(const custom::Point& p) {
        //     this->intensity = p.intensity;   // or p.reflectivity?
        //     this->range = this->norm();      // or p.range?
        // }

        void Point::pass_attributes(const Point& attributes) {
            this->intensity = attributes.intensity;
            this->range = attributes.range;
        }