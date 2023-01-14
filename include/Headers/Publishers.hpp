namespace Processor {
    void fill(pcl::PointCloud<full_info::Point>&, const Points&);
}

class Publishers {
    public:
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr states_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vel_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr full_pcl_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr planes_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_pub;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        double last_transform_time = -1;

        Publishers() {
            this->only_couts = true;
        }

        Publishers(rclcpp::Node::SharedPtr node) {
            this->state_pub = node->create_publisher<nav_msgs::msg::Odometry>("/limovelo/state", 1000); 
            this->states_pub = node->create_publisher<geometry_msgs::msg::PoseArray>("/limovelo/states", 1000); 

            this->vel_pub = node->create_publisher<nav_msgs::msg::Odometry>("/limovelo/vel", 1000); 
            this->yaw_pub = node->create_publisher<std_msgs::msg::Float32>("/limovelo/yaw", 1000); 
            
            this->pcl_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/limovelo/pcl", 1000); 
            this->full_pcl_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/limovelo/full_pcl", 1000); 

            this->gt_pub = node->create_publisher<nav_msgs::msg::Odometry>("/limovelo/gt", 1000);

            this->planes_pub = node->create_publisher<geometry_msgs::msg::PoseArray>("/limovelo/planes", 1000);
            this->only_couts = false;

            this->tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*node);
        }

        void state(const State& state, bool couts) {
            if (not this->only_couts) this->publish_state(state);
            if (couts) this->cout_state(state);
        }

        void states(const States& states) {
            this->publish_states(states);
        }

        void planes(const Planes& planes) {
            this->publish_planes(planes);
        }

        void pointcloud(Points& points, bool part=false) {
            if (points.empty()) return;
            
            pcl::PointCloud<full_info::Point> pcl;
            pcl.header.frame_id = "map";
            pcl.header.stamp = Conversions::sec2Microsec(points.back().time);
            Processor::fill(pcl, points);

            if (part) this->publish_pcl(pcl, this->pcl_pub);
            else this->publish_pcl(pcl, this->full_pcl_pub);
        }

        void rottransl(const RotTransl& RT) {
            this->cout_rottransl(RT);
        }

        void t1_t2(const Points& points, const IMUs& imus, const States& states, double t1, double t2) {
            this->cout_t1_t2(points, imus, states, t1, t2);
        }

        void tf(const State& state) {
            if (state.time <= this->last_transform_time) return;
            this->send_transform(state);
            this->last_transform_time = state.time;
        }

        void extrinsics(const State& state) {
            this->cout_extrinsics(state);
        }

    private:
        bool only_couts;

        void cout_extrinsics(const State& state) {
            std::cout << "t:" << std::endl;
            std::cout << state.I_Rt_L().t.transpose() << std::endl;
            std::cout << "R:" << std::endl;
            std::cout << state.I_Rt_L().R << std::endl;
            std::cout << "-----------" << std::endl;
        }

        void publish_planes(const Planes& planes) {
            geometry_msgs::msg::PoseArray normalPoseArray;
            normalPoseArray.header.frame_id = "map";
                
            for (Plane plane : planes) {
                Point point_world = plane.centroid;
                Normal n = plane.n;

                geometry_msgs::msg::Pose normalPose;
                normalPose.position.x = point_world.x;
                normalPose.position.y = point_world.y;
                normalPose.position.z = point_world.z;

                double NORM = std::sqrt(n.A*n.A + n.B*n.B + n.C*n.C);
                
                normalPose.orientation.x = 0;
                normalPose.orientation.y = -n.C;
                normalPose.orientation.z = n.B;
                normalPose.orientation.w = NORM + n.A;

                normalPoseArray.poses.push_back(normalPose);
            }

            if (this->planes_pub->get_subscription_count() > 0) this->planes_pub->publish(normalPoseArray);
        }

        void send_transform(const State& X) {
            geometry_msgs::msg::TransformStamped t;
            tf2::Quaternion q;

            t.header.stamp = rclcpp::Time(Conversions::sec2Nanosec(X.time));
            t.header.frame_id = "map";
            t.child_frame_id = "body";

            t.transform.translation.x = X.pos(0);
            t.transform.translation.y = X.pos(1);
            t.transform.translation.z = X.pos(2);
            
            Eigen::Quaternionf q_from_R(X.R);
            t.transform.rotation.x = q_from_R.x();
            t.transform.rotation.y = q_from_R.y();
            t.transform.rotation.z = q_from_R.z();
            t.transform.rotation.w = q_from_R.w();

            
            this->tf_broadcaster->sendTransform(t);
        }

        void cout_rottransl(const RotTransl& RT) {
            std::cout << RT.R << std::endl;
            std::cout << RT.t.transpose() << std::endl;
        }

        void publish_pcl(const pcl::PointCloud<full_info::Point>& pcl, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub) {
            sensor_msgs::msg::PointCloud2 msg;
            msg.header.stamp = rclcpp::Time(Conversions::microsec2Nanosec(pcl.header.stamp));
            msg.header.frame_id = "map";
            pcl::toROSMsg(pcl, msg);
            if (pub->get_subscription_count() > 0) pub->publish(msg);
        }

        void publish_states(const States& states) {
            geometry_msgs::msg::PoseArray msg;
            msg.header.frame_id = "map";
            msg.header.stamp = rclcpp::Time(Conversions::sec2Nanosec(states.back().time));

            for (const State& state : states) {
                geometry_msgs::msg::Pose pose;

                pose.position.x = state.pos(0);
                pose.position.y = state.pos(1);
                pose.position.z = state.pos(2);

                Eigen::Quaternionf q(state.R * state.I_Rt_L().R);
                pose.orientation.x = q.x();
                pose.orientation.y = q.y();
                pose.orientation.z = q.z();
                pose.orientation.w = q.w();

                msg.poses.push_back(pose);
            }

            if (this->states_pub->get_subscription_count() > 0) this->states_pub->publish(msg);
        }

        void publish_state(const State& state) {
            nav_msgs::msg::Odometry msg;
            msg.header.stamp = rclcpp::Time(Conversions::sec2Nanosec(state.time));
            msg.header.frame_id = "map";

            msg.pose.pose.position.x = state.pos(0);
            msg.pose.pose.position.y = state.pos(1);
            msg.pose.pose.position.z = state.pos(2);

            Eigen::Quaternionf q(state.R * state.I_Rt_L().R);
            msg.pose.pose.orientation.x = q.x();
            msg.pose.pose.orientation.y = q.y();
            msg.pose.pose.orientation.z = q.z();
            msg.pose.pose.orientation.w = q.w();

            // Local velocities
            Eigen::Vector3f localVels = state.R.transpose()*state.vel;
            msg.twist.twist.linear.x = localVels(0);
            msg.twist.twist.linear.y = localVels(1);
            msg.twist.twist.linear.z = localVels(2);

            msg.twist.twist.angular.x = state.w(0);
            msg.twist.twist.angular.y = state.w(1);
            msg.twist.twist.angular.z = state.w(2);

            if (this->state_pub->get_subscription_count() > 0) this->state_pub->publish(msg);
        }

        void cout_state(const State& state) {
            std::cout << "---------" << std::endl;
            std::cout << "New pos: " << state.pos.transpose() << std::endl;
            std::cout << "New vel: " << state.vel.transpose() << std::endl;
            std::cout << "New R: " << std::endl << state.R << std::endl;
            std::cout << "---------" << std::endl;
        }

        void cout_t1_t2(const Points& points, const IMUs& imus, const States& states, double t1, double t2) {
            if (points.size() > 0 and imus.size() > 0 and states.size() > 0) {
                std::cout << "-----------" << std::endl;
                std::cout << std::setprecision(16) << "   " << t1 << " to " << t2 << std::endl;
                std::cout << std::setprecision(16) << "L: " << points.front().time << " -- " << points.back().time << " = " << points.size() << std::endl;
                std::cout << std::setprecision(16) << "I: " << imus.front().time << " -- " << imus.back().time << " = " << imus.size() << std::endl;
                std::cout << std::setprecision(16) << "X: " << states.front().time << " -- " << states.back().time << " = " << states.size() << std::endl;
            }
        }

};
