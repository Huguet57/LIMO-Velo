class Publishers {
    public:
        ros::Publisher state_pub;
        ros::Publisher vel_pub;
        ros::Publisher yaw_pub;
        ros::Publisher pcl_pub;

        Publishers() {
            this->only_couts = true;
        }

        Publishers(ros::NodeHandle& nh) {
            this->state_pub = nh.advertise<nav_msgs::Odometry>("/limovelo/state", 1000); 
            this->vel_pub = nh.advertise<nav_msgs::Odometry>("/limovelo/vel", 1000); 
            this->yaw_pub = nh.advertise<std_msgs::Float32>("/limovelo/yaw", 1000); 
            this->pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/limovelo/pcl", 1000); 
            this->only_couts = false;
        }

        void state(const State& state, bool couts) {
            if (not only_couts) publish_pos(state);
            if (not only_couts) publish_vels(state);
            if (couts) cout_state(state);
        }

        void pointcloud(PointCloud& pcl) {
            pcl.header.frame_id = "map";
            publish_pcl(pcl);
        }

        void rottransl(const RotTransl& RT) {
            cout_rottransl(RT);
        }

        void t1_t2(const Points& points, const IMUs& imus, const States& states, double t1, double t2) {
            cout_t1_t2(points, imus, states, t1, t2);
        }

    private:
        bool only_couts;

        void cout_rottransl(const RotTransl& RT) {
            std::cout << RT.R << std::endl;
            std::cout << RT.t.transpose() << std::endl;
        }

        void publish_pcl(const PointCloud& pcl) {
            sensor_msgs::PointCloud2 msg;
            msg.header.stamp = ros::Time(Conversions::microsec2Sec(pcl.header.stamp));
            msg.header.frame_id = "map";
            pcl::toROSMsg(pcl, msg);
            this->pcl_pub.publish(msg);
        }

        void publish_yaw(const State& state) {
            std_msgs::Float32 yaw;

            Eigen::Quaternionf q(state.R);
            double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
            double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
            yaw.data = std::atan2(siny_cosp, cosy_cosp);

            this->yaw_pub.publish(yaw);
        }

        void publish_pos(const State& state) {
            nav_msgs::Odometry msg;
            msg.header.stamp = ros::Time(state.time);
            msg.header.frame_id = "map";

            msg.pose.pose.position.x = state.pos(0);
            msg.pose.pose.position.y = state.pos(1);
            msg.pose.pose.position.z = state.pos(2);

            Eigen::Quaternionf q(state.R);
            msg.pose.pose.orientation.x = q.x();
            msg.pose.pose.orientation.y = q.y();
            msg.pose.pose.orientation.z = q.z();
            msg.pose.pose.orientation.w = q.w();

            this->state_pub.publish(msg);
        }

        void publish_vels(const State& state) {
            nav_msgs::Odometry msg;
            msg.header.stamp = ros::Time(state.time);
            msg.header.frame_id = "map";

            msg.pose.pose.orientation.x = state.vel(0);
            msg.pose.pose.orientation.y = state.vel(1);
            msg.pose.pose.orientation.z = state.vel(2);
            msg.pose.pose.orientation.w = 0.;

            this->vel_pub.publish(msg);
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