extern struct Params Config;

template <typename ContentType>
class Buffer {
    public:
        std::deque<ContentType> content;
        Buffer();

        void push(const ContentType& cnt);
        void pop_front();
        void pop_back();    
        ContentType front();
        ContentType back();
        bool empty();
        int size();
        void clear();
        void clear(TimeType t);
};

class Point {
    public:
        float x;
        float y;
        float z;
        TimeType time;
        float intensity;
        float range;

        explicit Point();

        explicit Point(const full_info::Point& p);
        explicit Point(const Eigen::Matrix<float, 3, 1>& p);
        
        // Delegate constructor (Eigen + attributes)
        explicit Point(const Eigen::Matrix<float, 3, 1>& p, const Point& attributes);
        
        // HESAI specific
            explicit Point(const hesai_ros::Point& p);
            explicit Point(const hesai_ros::Point& p, double time_offset);

        // Velodyne specific
            explicit Point(const velodyne_ros::Point& p);
            explicit Point(const velodyne_ros::Point& p, double time_offset);
        
        // Ouster specific
            explicit Point(const ouster_ros::Point& p);
            explicit Point(const ouster_ros::Point& p, double time_offset);

        // rslidar specific
            explicit Point(const rslidar_ros::Point& p);
            explicit Point(const rslidar_ros::Point& p, double time_offset);

        full_info::Point toPCL() const;
        Eigen::Matrix<float, 3, 1> toEigen() const;

        float norm() const;
        Eigen::Vector3d cross(const Eigen::Vector3d& v);

        friend Point operator*(const Eigen::Matrix<float, 3, 3>&, const Point&);
        friend Point operator+(const Point& p, const Eigen::Matrix<float, 3, 1> v);
        friend Point operator-(const Point& p, const Eigen::Matrix<float, 3, 1> v);
        friend std::ostream& operator<< (std::ostream& out, const Point& p);

    private:
        template <typename PointType>
        void set_XYZ(const PointType& p);
        void set_XYZ(const Eigen::Matrix<float, 3, 1>& p);

        template <typename PointType>
        void set_attributes(const PointType& p);

        // Ouster specific
        void set_attributes(const ouster_ros::Point& p);
        
        // Point::set_attributes(const custom::Point& p);

        void pass_attributes(const Point& attributes);
};

class IMU {
    public:
        Eigen::Vector3f a;
        Eigen::Vector3f w;
        Eigen::Quaternionf q;
        TimeType time;

        IMU();
        IMU(const sensor_msgs::msg::Imu::ConstPtr& msg);

        IMU(const sensor_msgs::msg::Imu& imu);
        IMU (const Eigen::Vector3f& a, const Eigen::Vector3f& w, double time);
        IMU (const Eigen::Vector3f& a, const Eigen::Vector3f& w, const Eigen::Quaternionf& q, double time);
        IMU (double time);
        bool has_orientation();
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

        State();
        State(double time);
        State(const state_ikfom& s, double time);

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

        RotTransl(const State& S);
        RotTransl(const Eigen::Matrix3f& dR, const Eigen::Vector3f& dt);
        RotTransl inv();

        friend RotTransl operator* (const RotTransl&, const RotTransl&);
        friend Point operator* (const RotTransl&, const Point& p);
        friend Points operator* (const RotTransl&, const Points&);
};

class Normal {
    public:
        float A, B, C, D;

        Normal();
        Normal(const Eigen::Matrix<float, 4, 1>& ABCD);

        Eigen::Matrix<float, 3, 1> vect() const;
        friend Eigen::Matrix<double, 3, 1> operator* (const Eigen::Matrix<double, 3, 3>&, const Normal&);    
};

class Plane {
    public:
        bool is_plane;
        Point centroid;
        Normal n;
        
        Plane();
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

        Match(const Point& p, const Plane& H);
        
        bool is_chosen();
};