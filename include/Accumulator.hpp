class Accumulator {
    public:
        Buffer<Point> BUFFER_L;
        Buffer<IMU> BUFFER_I;
        Buffer<State> BUFFER_X;

        Accumulator() {}

        // Receive from topics
        void receive_lidar(const PointCloud_msg&);
        void receive_imu(const IMU_msg&);
        
        // Empty buffers
        void empty_buffers();
        void empty_buffers(TimeType);
        void empty_lidar(TimeType);

};