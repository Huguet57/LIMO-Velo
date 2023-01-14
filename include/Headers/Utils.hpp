namespace Conversions {
    std::uint64_t sec2Microsec(double t);
    std::uint64_t sec2Nanosec(double t);
    double microsec2Sec(std::uint64_t t);
    std::uint64_t microsec2Nanosec(std::uint64_t t) ;
    double nanosec2Sec(std::uint32_t t);
    std::vector<float> double2floatVect(std::vector<double> v);
}

namespace Algorithms {
    template <typename Array>
    int binary_search(const Array& sorted_content, double t, bool desc=true) {
        //binary search the given array
        int high, mid, low;
        low = 0; high = sorted_content.size() - 1;
        double sign = desc ? 1 : -1;

        while(high >= low){
            mid = (low + high)/2;
            (sign*sorted_content[mid].time < sign*t) ? high = mid - 1 : low = mid + 1;
        }

        // Return the leftest value
        if (--high < 0) return 0;
        return high;
    }
}

#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0

namespace SO3Math {

    template<typename T, typename Ts>
    Eigen::Matrix<T, 3, 3> Exp(Eigen::Matrix<T, 3, 1> ang_vel, Ts dt)
    {
        T ang_vel_norm = ang_vel.norm();
        Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

        if (ang_vel_norm > 0.0000001)
        {
            Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
            Eigen::Matrix<T, 3, 3> K;

            K << SKEW_SYM_MATRX(r_axis);

            T r_ang = ang_vel_norm * dt;

            /// Roderigous Tranformation
            return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
        }
        else
        {
            return Eye3;
        }
    }

}

namespace R3Math {
    Eigen::Matrix<float, 4, 1> estimate_plane(const PointVector&);
    bool is_plane(const Eigen::Matrix<float, 4, 1>&, const PointVector&, const float&);
    Point centroid(const PointVector&);
}

template <class TimeT = std::chrono::microseconds,
          class ClockT = std::chrono::steady_clock>
class Timer
{
    using timep_t = typename ClockT::time_point;
    timep_t _start = ClockT::now(), _end = {};

public:
    void tick() { 
        _end = timep_t{}; 
        _start = ClockT::now(); 
    }
    
    void tock() { _end = ClockT::now(); }
    
    template <class TT = TimeT> 
    TT duration() const { 
        assert(_end != timep_t{} && "toc before reporting"); 
        return std::chrono::duration_cast<TT>(_end - _start); 
    }

    double count() const {
        return this->duration().count()/1e3;
    }
};

typedef Timer<std::chrono::microseconds, std::chrono::steady_clock> MicroTimer;