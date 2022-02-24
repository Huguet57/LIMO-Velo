extern struct Params Config;

template <typename ContentType>
class Buffer {
    public:
        std::deque<ContentType> content;
        Buffer() {}

        void push(const ContentType& cnt) {
            this->content.push_front(cnt);
        }

        void pop_front() {
            this->content.pop_front();
        }

        void pop_back() {
            this->content.pop_back();
        }
        
        ContentType front() {
            return this->content.front();
        }

        ContentType back() {
            return this->content.back();
        }
        
        bool empty() {
            return this->content.empty();
        }

        int size() {
            return this->content.size();
        }

        void clear() {
            return this->content.clear();
        }

        void clear(TimeType t) {
            auto* buffer = &this->content;
            while (buffer->size() > 0 and t >= buffer->back().time)
                this->pop_back();
        }
};

class Point {
    public:
        float x;
        float y;
        float z;
        TimeType time;
        float intensity;
        float range;

        Point() {}

        Point(const full_info::Point& p) {
            this->set_XYZ(p);
            this->set_attributes(p);
            this->time = p.timestamp;
        }
  
        Point(const Eigen::Matrix<float, 3, 1>& p) {
            this->set_XYZ(p);
        }

        // Delegate constructor (Eigen + attributes)
        Point(const Eigen::Matrix<float, 3, 1>& p, const Point& attributes) : Point(p) {
            this->pass_attributes(attributes);
            this->time = attributes.time;
        }

        // Delegate constructor for any point type
        template <typename PointType>
        Point(const PointType& p, double time_offset)
            // Construct point as usual first
            : Point (p)
        {
            // Then add the time offset to the Point's time
            this->time += time_offset;
        }

        // HESAI specific
            Point(const hesai_ros::Point& p) {
                this->set_XYZ(p);
                this->set_attributes(p);
                this->time = p.timestamp;
            }
        
        // Velodyne specific
            Point(const velodyne_ros::Point& p) {
                this->set_XYZ(p);
                this->set_attributes(p);
                
                // Time offset with respect to beginning of rotation, i.e. ~= [0, 0.1]
                if (Config.offset_begin) this->time = (double) p.time;
                // Time offset with respect to end of rotation, i.e. ~= [-0.1, 0]
                else this->time = Config.full_rotation_time + (double) p.time;
            }

        // Ouster specific
            Point(const ouster_ros::Point& p) {
                this->set_XYZ(p);
                this->set_attributes(p);

                // Time offset with respect to beginning of rotation, i.e. ~= [0, 0.1]
                if (Config.offset_begin) this->time = Conversions::nanosec2Sec(p.t);
                // Time offset with respect to end of rotation, i.e. ~= [-0.1, 0]
                else this->time = Config.full_rotation_time + Conversions::nanosec2Sec(p.t);
            }

        // Custom specific
            Point(const custom::Point& p) {
                this->set_XYZ(p);

                // -------------------------------------------
                //      View below to modify this method
                // -------------------------------------------
                this->set_attributes(p);

                // -------------------------------------------
                //      Choose the appropiate time field
                // -------------------------------------------
                this->time = p.timestamp; // or p.time
            }

        full_info::Point toPCL() const {
            full_info::Point p;
            p.x = this->x;
            p.y = this->y;
            p.z = this->z;
            p.timestamp = this->time;
            p.intensity = this->intensity;
            p.range = this->range;
            return p;
        }

        Eigen::Matrix<float, 3, 1> toEigen() const {
            return Eigen::Matrix<float, 3, 1>(this->x, this->y, this->z);
        }

        float norm() const {
            return this->toEigen().norm(); 
        }

        Eigen::Vector3d cross(const Eigen::Vector3d& v) {
            Eigen::Vector3d w = this->toEigen().cast<double> ();
            return w.cross(v);
        }

        friend Point operator*(const Eigen::Matrix<float, 3, 3>&, const Point&);
        friend Point operator+(const Point& p, const Eigen::Matrix<float, 3, 1> v);
        friend Point operator-(const Point& p, const Eigen::Matrix<float, 3, 1> v);
        friend std::ostream& operator<< (std::ostream& out, const Point& p);

    private:
        template <typename PointType>
        void set_XYZ(const PointType& p) {
            this->x = p.x;
            this->y = p.y;
            this->z = p.z;
        }

        void set_XYZ(const Eigen::Matrix<float, 3, 1>& p) {
            this->x = p(0);
            this->y = p(1);
            this->z = p(2);
        }

        template <typename PointType>
        void set_attributes(const PointType& p) {
            this->intensity = p.intensity;
            this->range = this->norm();
        }
        
        // Ouster specific
        void set_attributes(const ouster_ros::Point& p) {
            this->intensity = p.reflectivity;
            this->range = p.range;
        }

        // ---------------------------------------------------------------------------------------
        //      Uncomment and modify this if point type doesn't have 'intensity' as attribute
        // ---------------------------------------------------------------------------------------
        
        // Point::set_attributes(const custom::Point& p) {
        //     this->intensity = p.intensity;   // or p.reflectivity?
        //     this->range = this->norm();      // or p.range?
        // }

        void pass_attributes(const Point& attributes) {
            this->intensity = attributes.intensity;
            this->range = attributes.range;
        }
};

class IMU {
    public:
        Eigen::Vector3f a;
        Eigen::Vector3f w;
        TimeType time;

        IMU() : IMU (Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), 0.) {}
        IMU(const sensor_msgs::ImuConstPtr& msg) : IMU(*msg) {}

        IMU(const sensor_msgs::Imu& imu) {
            // Linear accelerations
            this->a(0) = imu.linear_acceleration.x;
            this->a(1) = imu.linear_acceleration.y;
            this->a(2) = imu.linear_acceleration.z;

            // Gyroscope
            this->w(0) = imu.angular_velocity.x;
            this->w(1) = imu.angular_velocity.y;
            this->w(2) = imu.angular_velocity.z;

            // Time
            this->time = imu.header.stamp.toSec();
        }

        IMU (const Eigen::Vector3f& a, const Eigen::Vector3f& w, double time) {
            this->a = a;
            this->w = w;
            this->time = time;
        }

        IMU (double time) : IMU (Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), time) {}
};

class State {
    public:
        // State
        Eigen::Matrix3f R;
        Eigen::Vector3f pos;
        Eigen::Vector3f vel;
        Eigen::Vector3f bw;
        Eigen::Vector3f ba;
        Eigen::Vector3f g;

        // Offsets
        Eigen::Matrix3f RLI;
        Eigen::Vector3f tLI;

        // Last controls
        TimeType time;
        Eigen::Vector3f a;
        Eigen::Vector3f w;

        // Noises
        Eigen::Vector3f nw;
        Eigen::Vector3f na;
        Eigen::Vector3f nbw;
        Eigen::Vector3f nba;

        State() : State (0.) {}

        State(const state_ikfom& s, const IMU& imu, double time) : State(s, time) {
            this->a = imu.a;
            this->w = imu.w;
        }

        State(const state_ikfom& s, double time) : State (time) {
            this->R = s.rot.toRotationMatrix().cast<float>();
            this->pos = s.pos.cast<float>();
            this->vel = s.vel.cast<float>();
            
            this->bw = s.bg.cast<float>();
            this->ba = s.ba.cast<float>();

            this->RLI = s.offset_R_L_I.toRotationMatrix().cast<float>();
            this->tLI = s.offset_T_L_I.cast<float>();
        }

        State(double time) {
            Eigen::Vector3f init_g = Eigen::Map<Eigen::Vector3f>(Config.initial_gravity.data(), 3);
            Eigen::Vector3f I_t_L = Eigen::Map<Eigen::Vector3f>(Config.I_Translation_L.data(), 3);
            Eigen::Matrix3f I_R_L = Eigen::Map<Eigen::Matrix3f>(Config.I_Rotation_L.data(), 3, 3).transpose();

            this->R = Eigen::Matrix3f::Identity();
            this->g = init_g;

            this->RLI = I_R_L;
            this->tLI = I_t_L;

            this->pos = Eigen::Vector3f::Zero();
            this->vel = Eigen::Vector3f::Zero();
            this->bw = Eigen::Vector3f::Zero();
            this->ba = Eigen::Vector3f::Zero();

            this->time = time;
            this->a = init_g;
            this->w = Eigen::Vector3f::Zero();

            this->nw = Eigen::Vector3f::Zero();
            this->na = Eigen::Vector3f::Zero();
            this->nbw = Eigen::Vector3f::Zero();
            this->nba = Eigen::Vector3f::Zero();
        }

        RotTransl I_Rt_L() const;
        RotTransl inv() const;

        void operator+= (const IMU& imu);
        friend Point operator* (const State& X, const Point& p);
        friend RotTransl operator* (const State& X, const RotTransl& RT);
        friend Points operator* (const State& X, const Points& points);
    private:
        // When propagating, we set noises = 0
        void propagate_f(IMU imu, float dt);
        void update(IMU imu);  
};

class RotTransl {
    public:
        Eigen::Matrix3f R;
        Eigen::Vector3f t;

        RotTransl(const State& S) {
            this->R = S.R;
            this->t = S.pos;
        }

        RotTransl(const Eigen::Matrix3f& dR, const Eigen::Vector3f& dt) {
            this->R = dR;
            this->t = dt;
        }

        RotTransl inv() {
            return RotTransl(
                this->R.transpose(),
                -this->R.transpose()*this->t
            );
        }

        friend RotTransl operator* (const RotTransl&, const RotTransl&);
        friend Point operator* (const RotTransl&, const Point& p);
        friend Points operator* (const RotTransl&, const Points&);
};

class Normal {
    public:
        float A, B, C, D;

        Normal() {}
        
        Normal(const Eigen::Matrix<float, 4, 1>& ABCD) {
            this->A = ABCD(0);
            this->B = ABCD(1);
            this->C = ABCD(2);
            this->D = ABCD(3);
        }

        Eigen::Matrix<float, 3, 1> vect() const {
            return Eigen::Matrix<float, 3, 1> (
                this->A,
                this->B,
                this->C
            );
        }

        friend Eigen::Matrix<double, 3, 1> operator* (const Eigen::Matrix<double, 3, 3>&, const Normal&);    
};

class Plane {
    public:
        bool is_plane;
        Point centroid;
        Normal n;
        
        Plane() {}
        Plane(const PointVector&, const std::vector<float>&);
        float dist_to_plane(const Point&) const;
        bool on_plane(const Point&);

    private:
        bool enough_points(const PointVector&);
        bool points_close_enough(const std::vector<float>&);
        void fit_plane(const PointVector&);
};

class Match {
    public:
        Point point;
        Plane plane;
        float distance;

        Match(const Point& p, const Plane& H) {
            this->point = p;
            this->plane = H;
            this->distance = H.dist_to_plane(p);
        }

        bool is_chosen();
};