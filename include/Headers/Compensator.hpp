class Compensator {
    public:
        Accumulator* Ap;
        Publishers output;

        // Delegate constructors
        Compensator(Accumulator& A)
            : Compensator(A, 1e-2) {};
        
        Compensator(Accumulator& A, Publishers& pub, double delta)
            : Compensator(A, delta) {
                this->output = pub;
            };

        PointCloud compensate(double t, bool timecode)  {
            if (timecode) {
                MicroTimer compensator_clock;
                compensator_clock.tick();
                    PointCloud compensated = this->compensate(t);
                compensator_clock.tock();
                ROS_INFO("Compensate: %f ms", compensator_clock.count());
                return compensated;
            } else {
                return this->compensate(t);
            }
        }

        PointCloud compensate(double t)  {
            return this->compensate(t-delta, t);
        }

        // Main constructor
        PointCloud compensate(double t1, double t2);
        PointCloud compensate(States& states, Points& points);

        States integrate_imus(double t1, double t2);

    private:
        double delta;

        States integrate_imus(States& states, const IMUs& imus, double t1, double t2);

        Compensator(
            Accumulator& A,
            double delta
        )  {
            this->Ap = &A;
            this->delta = delta;
        }
};