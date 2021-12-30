class Accumulator {
    public:
        Buffer<Point> BUFFER_L;
        Buffer<IMU> BUFFER_I;
        Buffer<State> BUFFER_X;

        double initial_time;
        double delta;

        // Add to buffer
        void add(State, double time=-1);
        void add(IMU, double time=-1);
        void add(Point, double time=-1);

        // Receive from topics
        void receive_lidar(const PointCloud_msg&);
        void receive_imu(const IMU_msg&);
        
        // Empty buffers
        void empty_buffers();
        void empty_buffers(TimeType);
        void empty_lidar(TimeType);

        ///////////////////////////////////////////////////

        // Get content given time intervals

        IMU get_next_imu(double t2);
        State get_prev_state(double t1);

        States get_states(double t1, double t2);
        Points get_points(double t1, double t2);
        IMUs get_imus(double t1, double t2);

        template <typename ContentType>
        int before_t(Buffer<ContentType>& source, double t) {
            return Algorithms::binary_search(source.content, t, true);
        }

        template <typename ArrayType>
        int before_t(const ArrayType& array, double t, bool desc=false) {
            return Algorithms::binary_search(array, t, desc);
        }

        int before_first_state(const IMUs& imus, const States& states) {
            return Algorithms::binary_search(imus, states.front().time, false);
        }

        bool ready();
        ros::Rate refine_delta(double t);

    private:
        bool is_ready = false;

        void push(const State&);
        void push(const IMU&);
        void push(const Point&);

        template <typename ContentType>
        std::deque<ContentType> get(Buffer<ContentType>& source, double t1, double t2) {
            std::deque<ContentType> result;
            int k_t2 = before_t(source, t2);

            // Get content between t1 from t2 sorted old to new
            for (int k = k_t2; k < source.content.size(); ++k) {
                ContentType cnt = source.content[k];
                if (t1 >= cnt.time) break;
                if (t2 >= cnt.time) result.push_front(cnt);
            }

            return result;
        }

        bool enough_imus();
        void set_initial_time();


    // Singleton pattern
    public:
        static Accumulator& getInstance() {
            static Accumulator* accum = new Accumulator();
            return *accum;
        }

    private:
        Accumulator() = default;

        // Delete copy/move so extra instances can't be created/moved.
        Accumulator(const Accumulator&) = delete;
        Accumulator& operator=(const Accumulator&) = delete;
        Accumulator(Accumulator&&) = delete;
        Accumulator& operator=(Accumulator&&) = delete;

};