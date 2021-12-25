class Compensator {
    public:
        Buffer<Point>* BUFFER_Lp;
        Buffer<IMU>* BUFFER_Ip;
        Buffer<State>* BUFFER_Xp;
        Publishers output;

        // Delegate constructors
        Compensator(Accumulator& A)
            : Compensator(A.BUFFER_L, A.BUFFER_I, A.BUFFER_X, 1e-2) {};

        Compensator(Accumulator& A, double delta)
            : Compensator(A.BUFFER_L, A.BUFFER_I, A.BUFFER_X, delta) {};
        
        Compensator(Accumulator& A, Publishers& pub, double delta)
            : Compensator(A.BUFFER_L, A.BUFFER_I, A.BUFFER_X, delta) {
                this->output = pub;
            };

        PointCloud compensate(double t)  {
            return this->compensate(t-delta, t);
        }

        // Main constructor
        PointCloud compensate(double t1, double t2);
    private:
        double delta;

        Compensator(
            Buffer<Point>& BL,
            Buffer<IMU>& BI,
            Buffer<State>& BX,
            double delta
        )  {
            this->BUFFER_Lp = &BL;
            this->BUFFER_Ip = &BI;
            this->BUFFER_Xp = &BX;
            this->delta = delta;
        }

        ///////////////////////////////////////////////////

        template <typename ContentType>
        int before_t(Buffer<ContentType>* source, double t) {
            return Algorithms::binary_search(source->content, t, true);
        }

        template <typename ArrayType>
        int before_t(const ArrayType& array, double t, bool desc=false) {
            return Algorithms::binary_search(array, t, desc);
        }

        int before_first_state(const IMUs& imus, const States& states) {
            return Algorithms::binary_search(imus, states.front().time, false);
        }

        template <typename ContentType>
        std::deque<ContentType> get(Buffer<ContentType>* source, double t1, double t2) {
            std::deque<ContentType> result;
            int k_t2 = before_t(source, t2);

            // Get content between t1 from t2 sorted old to new
            for (int k = k_t2; k < source->content.size(); ++k) {
                ContentType cnt = source->content[k];
                if (t1 >= cnt.time) break;
                if (t2 >= cnt.time) result.push_front(cnt);
            }

            return result;
        }

        IMU get_next_imu(double t2);
        State get_prev_state(double t1);

        States get_states(double t1, double t2);
        Points get_points(double t1, double t2);
        IMUs get_imus(double t1, double t2);

        ///////////////////////////////////////////////////

        States integrate_imus(States& states, const IMUs& imus, double t1, double t2);
        PointCloud compensate(States& states, Points& points);
};