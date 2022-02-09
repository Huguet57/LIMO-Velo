class PointCloudProcessor {
    
    // Given a PointCloud, process it to push to the buffer

    public:
        Points points;
        PointCloudProcessor(const PointCloud_msg&);

    private:
        Points velodynemsg2points(const PointCloud_msg&);
        Points hesaimsg2points(const PointCloud_msg&);

        double begin_time(const pcl::PointCloud<velodyne_ros::Point>&);
        double begin_time(const pcl::PointCloud<hesai_ros::Point>&);

        template <typename PointType> Points to_points(const typename pcl::PointCloud<PointType>&);

        Points downsample(const Points&);
        static bool time_sort(const Point&, const Point&);
        void sort_points(Points&);
};

void fill(pcl::PointCloud<velodyne_ros::Point>&, const Points& points);