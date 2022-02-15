// -------------------------------------------
//             To add to Common.hpp
// -------------------------------------------
// (Try to copy the ones on the LiDAR ROS driver)

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


// -------------------------------------------
//    To add to Objects.hpp -> class Point
// -------------------------------------------
    // No need to change since it depends on 'custom::Point'
    Point(const custom::Point& p);
        
// ---------------------------------------------------------
//  To add to PointClouds.hpp -> class PointCloudProcessor
// ---------------------------------------------------------
    // No need to change since it depends on 'custom::Point'
    double get_begin_time(const pcl::PointCloud<custom::Point>&);
