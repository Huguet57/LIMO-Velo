class Compensator {
    public:
        Publishers output;

        Compensator() {};
        
        Compensator(Publishers& pub)
            : Compensator() {
                this->output = pub;
            };

        // Main constructor
        PointCloud compensate(double t1, double t2, bool global=false);
        PointCloud compensate(States& states, Points& points, bool global=false);
        
        States integrate_imus(double t1, double t2);

    private:
        States integrate_imus(States& states, const IMUs& imus, double t1, double t2);
};