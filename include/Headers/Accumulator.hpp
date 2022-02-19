class Accumulator {
    public:
        Buffer<Point> BUFFER_L;
        Buffer<IMU> BUFFER_I;
        Buffer<State> BUFFER_X;

        double initial_time;

        // Add to buffer
            void add(State, double time=-1);
            void add(IMU, double time=-1);
            void add(Point, double time=-1);
            void add(Points);

        // Receive from topics
            void receive_lidar(const PointCloud_msg&);
            void receive_livox(const livox_ros_driver::CustomMsg::ConstPtr&);
            void receive_imu(const IMU_msg&);
        
        // Empty buffers
            void clear_buffers();
            void clear_buffers(TimeType);
            void clear_lidar(TimeType);

        // Get content given time intervals

            State get_prev_state(double t);
            IMU get_next_imu(double t);

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

        // Start/stop

            bool ready();
            bool ended(double t);

        // Time management
        
            double update_delta(const HeuristicParams&, double t);
            double latest_time();

    private:
        bool is_ready = false;

        void push(const State&);
        void push(const IMU&);
        void push(const Point&);

        template <typename ContentType>
        std::deque<ContentType> get(Buffer<ContentType>& source, double t1, double t2) {
            std::deque<ContentType> result;
            int k_t2 = before_t(source, t2);

            // Get content between t1 from t2 sorted new to old
            for (int k = k_t2; k < source.content.size(); ++k) {
                ContentType cnt = source.content[k];
                if (t1 > cnt.time) break;
                if (t2 >= cnt.time) result.push_front(cnt);
            }

            return result;
        }

        template <typename ContentType>
        ContentType get_next(Buffer<ContentType>& source, double t) {
            int k_t = before_t(source, t);

            // Get rightest content left to t (sorted new to old)
            for (int k = k_t; k < source.content.size(); ++k) {
                ContentType cnt = source.content[k];
                ContentType next_cnt = source.content[k - 1];
                if (t >= cnt.time) return next_cnt;
            }

            // If not content found, push an empty one at t
            this->add(ContentType(), t);
            return source.front();
        }

        template <typename ContentType>
        ContentType get_prev(Buffer<ContentType>& source, double t) {
            int k_t = before_t(source, t) + 1;

            // Get leftest State right to t (sorted new to old)
            for (int k = k_t; k < source.content.size(); ++k) {
                ContentType cnt = source.content[k];
                if (t > cnt.time) return cnt;
            }

            // If not a state found, push an empty one at t
            this->add(ContentType(), t);
            return source.front();
        }

        // Process LiDAR pointcloud message
        template <typename pcl_msg>
        Points process(const pcl_msg& msg);

        bool enough_imus();
        void set_initial_time();
        double interpret_heuristic(const HeuristicParams&, double t);

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