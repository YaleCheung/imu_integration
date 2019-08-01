#ifndef IMUSTATE_HHH 
#define IMUSTATE_HHH

#include <Eigen/Geometry>

typedef struct IMUState {
    IMUState() : 
        time_stamp(0),
        ang_v(Eigen::Vector3d(0, 0, 0)), 
        acc_v(Eigen::Vector3d(0, 0, 0)) {}

    IMUState(const IMUState& state) :
        time_stamp(state.time_stamp), 
        ang_v(state.ang_v),
        acc_v(state.acc_v) {}

    IMUState(double time_stamp, Eigen::Vector3d ang_v, Eigen::Vector3d acc_v) :
        time_stamp(time_stamp), 
        ang_v(ang_v),
        acc_v(acc_v) {}

    IMUState& operator=(const IMUState& state) {
        this->time_stamp = state.time_stamp;
        this->ang_v = state.ang_v;
        this->acc_v = state.acc_v;
        return *this;
    }
    
    double time_stamp;
    Eigen::Vector3d ang_v;
    Eigen::Vector3d acc_v;
} IMUState;

#endif // IMUSTATE_HHH
