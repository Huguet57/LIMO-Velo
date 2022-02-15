// -------------------------------------------
//      Add this to PointCloudProcessor.cpp
// -------------------------------------------

    double PointCloudProcessor::get_begin_time(const pcl::PointCloud<custom::Point>& pcl) {
        // Example: Points with relative time
        return Conversions::microsec2Sec(pcl.header.stamp) - pcl.points.back().time;
        
        // Example: Points with absolute time
        return 0.d;
    }

    // No need to change anything since it depends on 'custom::Point'
    Points PointCloudProcessor::custommsg2points(const PointCloud_msg& msg) {
        pcl::PointCloud<custom::Point>::Ptr raw_pcl(new pcl::PointCloud<custom::Point>());
        pcl::fromROSMsg(*msg, *raw_pcl);
        return this->to_points(*raw_pcl);
    }

    // Add this line to the PointCloudProcessor::msg2points method
    if (Config.LiDAR_type == LIDAR_TYPE::Custom) return this->custommsg2points(msg);

// -------------------------------------------
//           Add this to Objects.cpp
// -------------------------------------------

    Point::Point(const custom::Point& p) {
        // -------------------------------------------
        //   View below to modify these two methods
        // -------------------------------------------
        this->set_XYZ(p);    
        this->set_attributes(p);

        // -------------------------------------------
        //      Choose the appropiate time field
        // -------------------------------------------
        this->time = p.timestamp; // or p.time
    }

    // ---------------------------------------------------------------------------------------
    //      Uncomment and modify this if point type doesn't have 'intensity' as attribute
    // ---------------------------------------------------------------------------------------
    
        // Point::set_attributes(const custom::Point& p) {
        //     this->intensity = p.intensity;   // or p.i?
        //     this->range = this->norm();      // or p.range?
        // }

    // ---------------------------------------------------------------------------------------
    //      Uncomment and modify this if point type doesn't have 'x', 'y', 'z' as attributes
    // ---------------------------------------------------------------------------------------
        
        // Point::set_XYZ(const custom::Point& p) {
        //     this->x = p.x;
        //     this->y = p.y;
        //     this->z = p.z;
        // }