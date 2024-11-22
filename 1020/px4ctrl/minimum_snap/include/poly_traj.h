#ifndef POLY_TRAJ_H
#define POLY_TRAJ_H

#include <ros/ros.h>
#include <Eigen/Eigen>

class PolyTrajClass {
public:
    PolyTrajClass() {
        Init(3, 1.0, 1.0);
    }
    PolyTrajClass(int order, double v_max, double a_max) {
        Init(order, v_max, a_max);
    }
    ~PolyTrajClass(){}

    void Init(int order, double v_max, double a_max) {
        order_ = order;
        v_max_ = v_max;
        a_max_ = a_max;
    }

    void AllocateTime(const Eigen::MatrixXd &waypoints, Eigen::VectorXd &time);
    void TrajGenerate(const Eigen::MatrixXd &pos, const Eigen::Vector3d &start_vel,
                      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &start_acc,
                      const Eigen::Vector3d &end_acc, const Eigen::VectorXd &time);
    
    double TotleTime(void) {
        double t_all = 0;
        for (int i = 0; i < time_.rows(); i++) {
            t_all += time_[i];
        }
        return t_all;
    }

    Eigen::Vector3d Getposition(double t) {
        Eigen::Vector3d pos(0, 0, 0);
        int m = 0;
        if (t < TotleTime()) {
            for (int i = 0; i < time_.rows(); i++) {
                if (t < time_[i]) {
                    m = i;
                    break;
                }
                t -= time_[i];
            }
        } else {
            m = time_.size() - 1;
            t = time_[m];
        }
        for (int n = 0; n < order_*2; n++) {
            double time = std::pow(t, n);
            pos.x() += poly_coeff_mat_(m, n) * time;
            pos.y() += poly_coeff_mat_(m, n+order_*2) * time;
            pos.z() += poly_coeff_mat_(m, n+order_*2*2) * time;
        }
        return pos;
    }

    Eigen::Vector3d GetVelocity(double t)
    {
        Eigen::Vector3d vel(0, 0, 0);
        int m = 0;
        if (t < TotleTime()) {
            for (int i = 0; i < time_.rows(); i++) {
                if (t < time_[i]) {
                    m = i;
                    break;
                }
                t -= time_[i];
            }
        } else {
            m = time_.size() - 1;
            t = time_[m];
        }
        for (int n = 1; n < order_*2; n++) {
            double time = std::pow(t, n - 1);
            vel.x() += n * poly_coeff_mat_(m, n) * time;
            vel.y() += n * poly_coeff_mat_(m, n+order_*2) * time;
            vel.z() += n * poly_coeff_mat_(m, n+order_*2*2) * time;
        }
        return vel;
    }

    Eigen::Vector3d GetAcceleration(double t)
    {
        Eigen::Vector3d acc(0, 0, 0);
        int m = 0;
        if (t < TotleTime()) {
            for (int i = 0; i < time_.rows(); i++) {
                if (t < time_[i]) {
                    m = i;
                    break;
                }
                t -= time_[i];
            }
        } else {
            m = time_.size() - 1;
            t = time_[m];
        }
        for (int n = 2; n < order_*2; n++) {
            double time = std::pow(t, n - 2);
            acc.x() += n * (n - 1) * poly_coeff_mat_(m, n) * time;
            acc.y() += n * (n - 1) * poly_coeff_mat_(m, n+order_*2) * time;
            acc.z() += n * (n - 1) * poly_coeff_mat_(m, n+order_*2*2) * time;
        }
        return acc;
    }

    Eigen::Vector3d GetJerk(double t)
    {
        Eigen::Vector3d jerk(0, 0, 0);
        int m = 0;
        if (t < TotleTime()) {
            for (int i = 0; i < time_.rows(); i++) {
                if (t < time_[i]) {
                    m = i;
                    break;
                }
                t -= time_[i];
            }
        } else {
            m = time_.size() - 1;
            t = time_[m];
        }
        for (int n = 3; n < order_*2; n++) {
            double time = std::pow(t, n - 3);
            jerk.x() += n * (n - 1) * (n - 2) * poly_coeff_mat_(m, n) * time;
            jerk.y() += n * (n - 1) * (n - 2) * poly_coeff_mat_(m, n+order_*2) * time;
            jerk.z() += n * (n - 1) * (n - 2) * poly_coeff_mat_(m, n+order_*2*2) * time;
        }
        return jerk;
    }

private:
    int order_;
    double v_max_;
    double a_max_;

    Eigen::MatrixXd poly_coeff_mat_;
    Eigen::VectorXd time_;
};

#endif
