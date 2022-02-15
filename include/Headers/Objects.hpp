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
        }

        Point(const hesai_ros::Point& p) {
            this->set_XYZ(p);
            this->set_attributes(p);
        }

        Point(const velodyne_ros::Point& p) {
            this->set_XYZ(p);
            this->set_attributes(p);
        }

        // Delegate constructor for HESAI (absolute time)
        Point(const hesai_ros::Point& p, double begin_time) : Point (p) {}
        
        // Delegate constructor for Velodyne (relative time)
        Point(const velodyne_ros::Point& p, double begin_time) : Point (p) {
            this->time += begin_time;
        }

        Point(const Eigen::Matrix<float, 3, 1>& p) {
            this->set_XYZ(p);
        }

        // Delegate constructor
        Point(const Eigen::Matrix<float, 3, 1>& p, const Point& attributes) : Point(p) {
            this->set_attributes(attributes);
        }
        
        full_info::Point toPCL() const {
            full_info::Point p;
            p.x = this->x;
            p.y = this->y;
            p.z = this->z;
            p.timestamp = this->time;
            p.intensity = this->intensity;
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
        void set_XYZ(const full_info::Point& p) {
            this->x = p.x;
            this->y = p.y;
            this->z = p.z;
        }

        void set_XYZ(const velodyne_ros::Point& p) {
            this->x = p.x;
            this->y = p.y;
            this->z = p.z;
        }

        void set_XYZ(const hesai_ros::Point& p) {
            this->x = p.x;
            this->y = p.y;
            this->z = p.z;
        }

        void set_XYZ(const Eigen::Matrix<float, 3, 1>& p) {
            this->x = p(0);
            this->y = p(1);
            this->z = p(2);
        }

        void set_attributes(const velodyne_ros::Point& p) {
            this->time = (double) p.time;
            this->intensity = p.intensity;
            this->range = this->norm();
        }

        void set_attributes(const hesai_ros::Point& p) {
            this->time = p.timestamp;
            this->intensity = p.intensity;
            this->range = this->norm();
        }

        void set_attributes(const full_info::Point& p) {
            this->time = p.timestamp;
            this->intensity = p.intensity;
            this->range = this->norm();
        }

        void set_attributes(const Point& point_attributes) {
            this->time = point_attributes.time;
            this->intensity = point_attributes.intensity;
            this->range = point_attributes.range;
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
            Eigen::Matrix3f I_R_L = Eigen::Map<Eigen::Matrix3f>(Config.I_Rotation_L.data(), 3, 3);

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