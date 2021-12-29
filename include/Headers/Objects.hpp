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
        
        void empty() {
            this->content.empty();
        }

        int size() {
            return this->content.size();
        }

        void empty(TimeType t) {
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

        Point() {}

        Point(const PointType& p) {
            this->x = p.x;
            this->y = p.y;
            this->z = p.z;
            this->time = p.time;
        }

        Point(const PointType& p, double begin_time) {
            this->x = p.x;
            this->y = p.y;
            this->z = p.z;
            this->time = begin_time + p.time;
        }

        Point(float x, float y, float z, double time=0) {
            this->x = x;
            this->y = y;
            this->z = z;
            this->time = time;
        }

        Point(const Eigen::Matrix<float, 3, 1>& p, double t) {
            this->x = p(0);
            this->y = p(1);
            this->z = p(2);
            this->time = t;
        }

        PointType toPCL() const {
            PointType p;
            p.x = this->x;
            p.y = this->y;
            p.z = this->z;
            p.time = this->time;
            return p;
        }

        Eigen::Matrix<float, 3, 1> toEigen() const {
            return Eigen::Matrix<float, 3, 1>(this->x, this->y, this->z);
        }

        Eigen::Vector3d cross(const Eigen::Vector3d& v) {
            Eigen::Vector3d w = this->toEigen().cast<double> ();
            
            Eigen::Matrix3d Wx;
            Wx << 0.0,-w[2],w[1],w[2],0.0,-w[0],-w[1],w[0],0.0;

            return Wx * v;

            // return Eigen::Vector3d(
            //     w(1) * v(2) + w(2) * v(1),
            //     w(2) * v(0) + w(0) * v(2),
            //     w(0) * v(1) + w(1) * v(0)
            // );
        }

        friend Point operator*(const Eigen::Matrix<float, 3, 3>&, const Point&);
        friend Point operator+(const Point& p, const Eigen::Matrix<float, 3, 1> v);
        friend Point operator-(const Point& p, const Eigen::Matrix<float, 3, 1> v);
        friend std::ostream& operator<< (std::ostream& out, const Point& p);
};

class IMU {
    public:
        Eigen::Vector3f a;
        Eigen::Vector3f w;
        TimeType time;

        IMU() {}
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

        IMU(const Eigen::Vector3f& a, const Eigen::Vector3f& w, double time) {
            this->a = a;
            this->w = w;
            this->time = time;
        }

        friend std::ostream& operator<< (std::ostream& out, const IMU& imu);
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

        /////////////////////

        State(const state_ikfom& s, double time) {
            this->R = s.rot.toRotationMatrix().cast<float>();
            this->pos = s.pos.cast<float>();

            this->vel = s.vel.cast<float>();
            this->g = Eigen::Vector3f(0.,0.,-9.807); // TODO?
            
            this->bw = s.bg.cast<float>();
            this->ba = s.ba.cast<float>();

            this->RLI = s.offset_R_L_I.toRotationMatrix().cast<float>();
            this->tLI = s.offset_T_L_I.cast<float>();

            // TODO?
            this->nw = Eigen::Vector3f(0.,0.,0.);
            this->na = Eigen::Vector3f(0.,0.,0.);
            this->nbw = Eigen::Vector3f(0.,0.,0.);
            this->nba = Eigen::Vector3f(0.,0.,0.);

            this->time = time;
            this->a = Eigen::Vector3f(0., 0., 0.);
            this->w = Eigen::Vector3f(0., 0., 0.);
        }

        State(double time) {
            this->R = Eigen::Matrix3f::Identity();
            this->g = Eigen::Vector3f(0.,0.,-9.807);
            
            // Xaloc's custom LiDAR-IMU offsets
            this->RLI = Eigen::Matrix3f::Identity();
            this->RLI(1,1) = this->RLI(2,2) = -1;
            this->tLI = Eigen::Vector3f(0.9,0.,0.);

            this->pos = Eigen::Vector3f(0.,0.,0.);
            this->vel = Eigen::Vector3f(0.,0.,0.);
            this->bw = Eigen::Vector3f(0.,0.,0.);
            this->ba = Eigen::Vector3f(0.,0.,0.);

            this->time = time;
            this->a = Eigen::Vector3f(0., 0., 0.);
            this->w = Eigen::Vector3f(0., 0., 0.);

            this->nw = Eigen::Vector3f(0.,0.,0.);
            this->na = Eigen::Vector3f(0.,0.,0.);
            this->nbw = Eigen::Vector3f(0.,0.,0.);
            this->nba = Eigen::Vector3f(0.,0.,0.);
        }

        RotTransl I_Rt_L() const;

        void operator+= (const IMU& imu);
        friend RotTransl operator- (const State& st, const State& s0);
        friend Point operator* (const State& X, const Point& p);
        friend RotTransl operator* (const State& X, const RotTransl& RT);
        friend PointCloud operator* (const State& X, const PointCloud& pcl);
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
            return RotTransl(this->R.transpose(), -this->t);
        }

        friend RotTransl operator* (const RotTransl&, const RotTransl&);
        friend Point operator* (const RotTransl&, const Point& p);
        friend PointCloud operator* (const RotTransl&, const PointCloud&);
};

class Plane {
    private:
        int NUM_MATCH_POINTS = 5;
        const float MAX_DIST = 2.;

    public:
        bool is_plane;
        Point centroid;
        Normal n;
        float distance;
        
        Plane(const PointType&, const PointTypes&, const std::vector<float>&);
        template <typename AbstractPoint> bool on_plane(const AbstractPoint& p, float& res);

    private:
        bool enough_points(const PointTypes&);
        bool points_close_enough(const std::vector<float>&);

        void calculate_attributes(const PointType&, const PointTypes&);
        template<typename T> bool estimate_plane(Eigen::Matrix<T, 4, 1> &, const PointTypes &, const T &);
};

// struct Normal
    Eigen::Matrix<double, 3, 1> operator* (const Eigen::Matrix<double, 3, 3>&, const Normal&);