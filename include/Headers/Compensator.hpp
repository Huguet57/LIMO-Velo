class Compensator {
    public:
        Compensator() {};

        // Main constructor
        PointCloud compensate(double t1, double t2, bool global=false);
        PointCloud compensate(States& states, Points& points, bool global=false);
        
        States integrate_imus(double t1, double t2);
        PointCloud downsample(const PointCloud&);

    private:
        States integrate_imus(States& states, const IMUs& imus, double t1, double t2);
        PointCloud voxelgrid_downsample(const PointCloud&);
};