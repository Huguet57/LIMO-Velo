class Compensator {
    public:
        Compensator() {};

        // Main constructor
        Points compensate(double t1, double t2);
        Points compensate(const States& states, const State& Xt2, const Points& points);
        
        States path(double t1, double t2);
        Points downsample(const Points&);

    private:
        State get_t2(const States&, double t2);
        States upsample(const States&, const IMUs&);

        Points voxelgrid_downsample(const Points&);
        Points onion_downsample(const Points&);
};