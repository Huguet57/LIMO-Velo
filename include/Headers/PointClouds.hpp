class PointCloudProcessor {
    
    // Given a PointCloud, process it to push to the buffer

    public:
        Points points;
        PointCloudProcessor(const PointCloud_msg&);

    private:
        Points to_points(const PointCloud::Ptr&);
        Points downsample(const Points&);
        static bool time_sort(const Point&, const Point&);
        void sort_points(Points&);
};

void operator+= (PointCloud&, const Point&);