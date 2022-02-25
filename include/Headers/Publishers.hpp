namespace Processor {
    void fill(pcl::PointCloud<full_info::Point>&, const Points&);
}

class Publishers {
    public:
        ros::Publisher state_pub;
        ros::Publisher states_pub;
        ros::Publisher vel_pub;
        ros::Publisher yaw_pub;
        ros::Publisher pcl_pub;
        ros::Publisher full_pcl_pub;
        ros::Publisher planes_pub;
        ros::Publisher gt_pub;

        double last_transform_time = -1;

        Publishers() {
            this->only_couts = true;
        }

        Publishers(ros::NodeHandle& nh) {
            this->state_pub = nh.advertise<nav_msgs::Odometry>("/limovelo/state", 1000); 
            this->states_pub = nh.advertise<geometry_msgs::PoseArray>("/limovelo/states", 1000); 

            this->vel_pub = nh.advertise<nav_msgs::Odometry>("/limovelo/vel", 1000); 
            this->yaw_pub = nh.advertise<std_msgs::Float32>("/limovelo/yaw", 1000); 
            
            this->pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/limovelo/pcl", 1000); 
            this->full_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/limovelo/full_pcl", 1000); 

            this->gt_pub = nh.advertise<nav_msgs::Odometry>("/limovelo/gt", 1000);

            this->planes_pub = nh.advertise<geometry_msgs::PoseArray>("/limovelo/planes", 1000);
            this->only_couts = false;
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
            geometry_msgs::PoseArray normalPoseArray;
            normalPoseArray.header.frame_id = "map";
                
            for (Plane plane : planes) {
                Point point_world = plane.centroid;
                Normal n = plane.n;

                geometry_msgs::Pose normalPose;
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

            this->planes_pub.publish(normalPoseArray);
        }

        void send_transform(const State& X) {
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            tf::Quaternion q;

            transform.setOrigin(tf::Vector3(X.pos(0), \
                                            X.pos(1), \
                                            X.pos(2)));
            
            Eigen::Quaternionf q_from_R(X.R);
            q.setW(q_from_R.w());
            q.setX(q_from_R.x());
            q.setY(q_from_R.y());
            q.setZ(q_from_R.z());
            transform.setRotation(q);
            
            br.sendTransform(tf::StampedTransform(transform, ros::Time(X.time), "map", "body"));
        }

        void cout_rottransl(const RotTransl& RT) {
            std::cout << RT.R << std::endl;
            std::cout << RT.t.transpose() << std::endl;
        }

        void publish_pcl(const pcl::PointCloud<full_info::Point>& pcl, ros::Publisher pub) {
            sensor_msgs::PointCloud2 msg;
            msg.header.stamp = ros::Time(Conversions::microsec2Sec(pcl.header.stamp));
            msg.header.frame_id = "map";
            pcl::toROSMsg(pcl, msg);
            pub.publish(msg);
        }

        void publish_states(const States& states) {
            geometry_msgs::PoseArray msg;
            msg.header.frame_id = "map";
            msg.header.stamp = ros::Time(states.back().time);

            for (const State& state : states) {
                geometry_msgs::Pose pose;

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

            this->states_pub.publish(msg);
        }

        void publish_state(const State& state) {
            nav_msgs::Odometry msg;
            msg.header.stamp = ros::Time(state.time);
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

            this->state_pub.publish(msg);
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
