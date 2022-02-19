class PointCloudProcessor {
    
    // Given a PointCloud, process it to push to the buffer

    public:
        PointCloudProcessor() = default;
        Points msg2points(const PointCloud_msg&);
        Points msg2points(const livox_ros_driver::CustomMsg::ConstPtr&);
        Points downsample(const Points&);
        Points sort_points(const Points&);

    private:
        // Velodyne specific
            Points velodynemsg2points(const PointCloud_msg&);
            double get_begin_time(const pcl::PointCloud<velodyne_ros::Point>&);
        
        // HESAI specific
            Points hesaimsg2points(const PointCloud_msg&);
            double get_begin_time(const pcl::PointCloud<hesai_ros::Point>&);
        
        // Ouster specific
            Points oustermsg2points(const PointCloud_msg&);
            double get_begin_time(const pcl::PointCloud<ouster_ros::Point>&);

        // Livox specific
            Points livoxmsg2points(const livox_ros_driver::CustomMsg::ConstPtr&);
            double get_begin_time(const livox_ros_driver::CustomMsg&);

        // Custom specific
            Points custommsg2points(const PointCloud_msg&);
            double get_begin_time(const pcl::PointCloud<custom::Point>&);
        
        Points to_points(const livox_ros_driver::CustomMsg&);
        template <typename PointType> Points to_points(const typename pcl::PointCloud<PointType>&);

        Points temporal_downsample(const Points&);
        static bool time_sort(const Point&, const Point&);
};