class PointCloudProcessor {
    
    // Given a PointCloud, process it to push to the buffer

    public:
        Points points;
        PointCloudProcessor(const PointCloud_msg&, Buffer<Point>&);

    private:
        PointCloud downsample(const PointCloud::Ptr&);
        void add2Buffer(const PointCloud&, Buffer<Point>&);
        static bool time_sort(const PointType&, const PointType&);
        void sort_points(std::vector<PointType, Eigen::aligned_allocator<PointType>>&);
};

void operator+= (PointCloud&, const Point&);