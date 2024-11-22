#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "param.h"
#include "blackboard.h"
#include <queue>

struct DesireState_t {
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    Eigen::Quaterniond q;
    double yaw;
    double yaw_rate;
};

struct CtrlOutput_t {
    Eigen::Quaterniond q;     // Orientation of the body frame with respect to the world frame
    Eigen::Vector3d bodyrates; // Body rates in body frame[rad/s]
	double thrust;            // Collective mass normalized thrust
};

class ControllerClass {
public:
    ControllerClass(){}
    ~ControllerClass(){}
    void Reset(Param &param) {
        // P_ = 1e6;
        P_ = 1.5;
        thr_scale_compensate_ = 1;
        thr2acc_ = param.gra_ / param.thr_model_.hover_percentage;
        Gravity_ = Eigen::Vector3d(0.0, 0.0, -param.gra_);
    }

    void RotorDragLoop(const DesireState_t &des, CtrlOutput_t &u, Blackboard &bd, Param &param);
    bool estimateThrustModel(const Eigen::Vector3d &est_a, const double voltage, const Param &param);

private:
    double ComputeThrust(const Eigen::Quaterniond &est_q, const Eigen::Vector3d &est_v,
                         const Eigen::Vector3d &des_acc, const Param &param, const double voltage);
    Eigen::Quaterniond ComputeAttitude(const Eigen::Quaterniond &est_q, const Eigen::Vector3d &des_acc, const double des_yaw);
    Eigen::Vector3d ComputeBodyrate(const Eigen::Quaterniond &des_q, const Eigen::Quaterniond &est_q, const Param &param);
    
    Eigen::Vector3d computeRobustBodyXAxis(const Eigen::Vector3d &x_B_org, const Eigen::Vector3d &x_C,
                                           const Eigen::Vector3d &y_C, const Eigen::Quaterniond &est_q);
    double AccurateThrustAccMapping(const double des_acc_z, double voltage, const Param &param);

    Eigen::Vector3d VecterClip(const Eigen::Vector3d vec, double min, double max) {
        Eigen::Vector3d result(0, 0, 0);
        for (int i = 0; i < 3; i++) {
            result(i) = vec(i);
            if(vec(i) > max) result(i) = max;
            if(vec(i) < min) result(i) = min;
        }
        return result;
    }

    std::queue<std::pair<ros::Time, double>> timed_thrust_;
    double thr_scale_compensate_;
    double thr2acc_;
    double P_;
    Eigen::Vector3d Gravity_;

    const double rho2_ = 0.998; // do not change
    const double kMinNormalizedCollectiveThrust_ = 3.0;
	// const double kAlmostZeroValueThreshold_ = 0.001;
	// const double kAlmostZeroThrustThreshold_ = 0.01;
};

#endif
