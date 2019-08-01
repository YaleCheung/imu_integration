#ifndef IMUIntegration_HHH
#define IMUIntegration_HHH

#include <vector>

#include "imu_state.h"

Eigen::Vector3d acc_gravity(0, 0, 9.81);


class IMUIntegration{
public:
    IMUIntegration() : 
        _init(false),
        _accu_rot(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        _last_pos(Eigen::Vector3d(0.0, 0.0, 0.0)),
        _last_v(Eigen::Vector3d(0.0, 0.0, 0.0)),
        _bias_acc(Eigen::Vector3d(0.0, 0.0, 0.0)),
        _bias_gyr(Eigen::Vector3d(0.0, 0.0, 0.0)),
        _noise_acc(Eigen::Vector3d(0.0, 0.0, 0.0)),
        _noise_gyr(Eigen::Vector3d(0.0, 0.0, 0.0)) { }

    void Integrate(IMUState cur_state) {
        // assume the init pose is horizontal
        if (! _init) {
            _last_state = cur_state;
            _init = true;
            return;
        } 
        // integration
        double dt = cur_state.time_stamp - _last_state.time_stamp;
        Eigen::Vector3d acc_ub_last = _accu_rot.conjugate().normalized() * (_last_state.acc_v - _bias_acc - _noise_acc) - acc_gravity;
        Eigen::Vector3d avg_ang_ub = (cur_state.ang_v + _last_state.ang_v) / 2.0 - _bias_gyr - _noise_gyr;
        Eigen::Quaterniond cur_rot = (Eigen::AngleAxisd(cur_state.ang_v(2) * dt, Eigen::Vector3d::UnitZ()) * 
                        Eigen::AngleAxisd(cur_state.ang_v(1) * dt, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(cur_state.ang_v(0) * dt, Eigen::Vector3d::UnitX())) * _accu_rot;
        Eigen::Vector3d acc_cur = cur_rot.conjugate().normalized() * (cur_state.acc_v - _bias_acc - _noise_acc) - acc_gravity;
        Eigen::Vector3d avg_acc_ub = (acc_cur + acc_ub_last) / 2.0;

        // update data;
        _last_pos = _last_pos + _last_v * dt + avg_acc_ub * dt * dt / 2.0;
        _last_v = _last_v + avg_acc_ub * dt;
        _accu_rot = cur_rot;
        _last_state = cur_state;
    }
    auto getPos() const { return _last_pos; }

    auto getV() const { return _last_v; } 

    void setBiasAcc(const Eigen::Vector3d& bias_acc) {
        _bias_acc = bias_acc;
    }

    void setBiasGyr(const Eigen::Vector3d& bias_gyr) {
        _bias_gyr = bias_gyr;
    }

    void setNoiseAcc(const Eigen::Vector3d& noise) {
        _noise_acc = noise;
    }

    void setNoiseGyr(const Eigen::Vector3d& noise) {
        _noise_gyr = noise;
    }
private:
    bool                  _init;
    IMUState              _last_state;
    Eigen::Quaterniond    _accu_rot;
    Eigen::Vector3d       _last_pos;
    Eigen::Vector3d       _last_v;

    Eigen::Vector3d       _bias_acc;
    Eigen::Vector3d       _bias_gyr;
    Eigen::Vector3d       _noise_acc;
    Eigen::Vector3d       _noise_gyr;
};

#endif //IMUIntegration_HHH
