#include "controller.h"

void ControllerClass::RotorDragLoop(const DesireState_t &des, CtrlOutput_t &u, Blackboard &bd, Param &param)
{
    double error_dis = std::fabs((des.p - bd.odom_.p).norm());
    if (error_dis > 2.0) { // check distance, if too far, UAV will be dangerous, error!
        ROS_ERROR("odom desire is far awat!!! odom may have failed!!!  exit   error_dis = %f",error_dis);
        u.bodyrates = Eigen::Vector3d(0, 0, 0);
        u.q = Eigen::Quaterniond(1, 0, 0, 0);
        u.thrust = 0.2;
        timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
        while (timed_thrust_.size() > 100) {
            timed_thrust_.pop();
        }
        return ;
    }

    // TODO Compute reference inputs that compensate for aerodynamic drag 

    // PD + acc feedforward + gravity compensate controller
    Eigen::Vector3d error_p = VecterClip(des.p - bd.odom_.p, -1, 1);
    Eigen::Vector3d error_v = VecterClip(des.v - bd.odom_.v, -1, 1);
    Eigen::Matrix3d Kp = Eigen::Vector3d(param.gain_.Kp0, param.gain_.Kp1, param.gain_.Kp2).asDiagonal();
    Eigen::Matrix3d Kv = Eigen::Vector3d(param.gain_.Kv0, param.gain_.Kv1, param.gain_.Kv2).asDiagonal();
    Eigen::Vector3d des_acc = Kp * error_p + Kv * error_v + des.a - Gravity_; // TODO drag

    u.thrust = ComputeThrust(bd.odom_.q, bd.odom_.v, des_acc, param, bd.battery_.volt); // u1
    Eigen::Quaterniond des_q = ComputeAttitude(bd.odom_.q, des_acc, des.yaw);

    if (param.bodyrate_flag_) {
        Eigen::Vector3d fdb_bodyrate = ComputeBodyrate(des_q, bd.odom_.q, param);
        u.bodyrates += fdb_bodyrate;
        u.bodyrates = VecterClip(u.bodyrates, -3*M_PI, 3*M_PI);   // limit bodyrates
        if (u.bodyrates.z() >  M_PI_2) u.bodyrates.z() =  M_PI_2;
        if (u.bodyrates.z() < -M_PI_2) u.bodyrates.z() = -M_PI_2;
    } else {
        // Align to FCU frame: R_W1Bdes = R_W1B * R_BW2 * R_W2Bdes
        u.q = bd.imu_.q * bd.odom_.q.inverse() * des_q;
    }
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
    while (timed_thrust_.size() > 100) {
        timed_thrust_.pop();
    }
}

Eigen::Quaterniond ControllerClass::ComputeAttitude(const Eigen::Quaterniond &est_q, const Eigen::Vector3d &des_acc, const double des_yaw)
{
    Eigen::Quaterniond q_head = Eigen::Quaterniond(Eigen::AngleAxisd(des_yaw, Eigen::Vector3d::UnitZ()));

    // Compute desired orientation
    const Eigen::Vector3d x_C = q_head * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d y_C = q_head * Eigen::Vector3d::UnitY();

    Eigen::Vector3d z_B;
    if (std::fabs(des_acc.norm()) < 1e-3) {
        z_B = est_q * Eigen::Vector3d::UnitZ();
    } else {
        z_B = des_acc.normalized();
    }
    Eigen::Vector3d x_B = y_C.cross(z_B).normalized();
    // Eigen::Vector3d x_B = computeRobustBodyXAxis(x_B_org, x_C, y_C, est_q);
    Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();
    Eigen::Matrix3d R_WB((Eigen::Matrix3d() << x_B, y_B, z_B).finished());
    return Eigen::Quaterniond(R_WB);
}

Eigen::Vector3d ControllerClass::computeRobustBodyXAxis(const Eigen::Vector3d &x_B_org, const Eigen::Vector3d &x_C,
                                                        const Eigen::Vector3d &y_C, const Eigen::Quaterniond &est_q)
{
    Eigen::Vector3d x_B = x_B_org;
    if (std::fabs(x_B.norm()) < 1e-3) {
        // if cross(y_C, z_B) == 0, they are collinear =>
        // every x_B lies automatically in the x_C - z_C plane
        // Project estimated body x-axis into the x_C - z_C plane
        const Eigen::Vector3d x_B_estimated = est_q * Eigen::Vector3d::UnitX();
        const Eigen::Vector3d x_B_projected = x_B_estimated - (x_B_estimated.dot(y_C)) * y_C;
        if (std::fabs(x_B_projected.norm()) < 1e-3) {
            // Not too much intelligent stuff we can do in this case but it should basically never occur
            x_B = x_C;
        } else {
            x_B = x_B_projected.normalized();
        }
    } else {
        x_B.normalize();
    }

    // if the quad is upside down, x_B will point in the "opposite" direction
    // of x_C => flip x_B (unfortunately also not the solution for our problems)
    if (x_B.dot(x_C) < 0.0) {
        x_B = -x_B;
    }
    return x_B;
}

Eigen::Vector3d ControllerClass::ComputeBodyrate(const Eigen::Quaterniond &des_q, const Eigen::Quaterniond &est_q, const Param &param)
{
    // Compute the error quaternion
    const Eigen::Quaterniond q_e = est_q.inverse() * des_q;

    // Compute desired body rates from control error
    Eigen::Vector3d bodyrates;
    if (q_e.w() >= 0) {
        bodyrates.x() =  2.0 * param.gain_.KAngR * q_e.x();
        bodyrates.y() =  2.0 * param.gain_.KAngP * q_e.y();
        bodyrates.z() =  2.0 * param.gain_.KAngY * q_e.z();
    } else {
        bodyrates.x() = -2.0 * param.gain_.KAngR * q_e.x();
        bodyrates.y() = -2.0 * param.gain_.KAngP * q_e.y();
        bodyrates.z() = -2.0 * param.gain_.KAngY * q_e.z();
    }

    return bodyrates;
}


double ControllerClass::ComputeThrust(const Eigen::Quaterniond &est_q, const Eigen::Vector3d &est_v,
                                      const Eigen::Vector3d &des_acc, const Param &param, const double voltage)
{
    double norm_thrust;
    const Eigen::Vector3d body_z_axis = est_q * Eigen::Vector3d::UnitZ();
    double des_acc_norm = des_acc.dot(body_z_axis);
    if (des_acc_norm < kMinNormalizedCollectiveThrust_) {
        des_acc_norm = kMinNormalizedCollectiveThrust_;
    }

    // This compensates for an acceleration component in thrust direction due to the square of the body-horizontal velocity.
    des_acc_norm -= param.rt_drag_.k_thrust_horz * (std::pow(est_v.x(), 2.0) + std::pow(est_v.y(), 2.0));

    if (param.thr_model_.accurate_thrust_model_flag) {
        norm_thrust = thr_scale_compensate_ * AccurateThrustAccMapping(des_acc_norm, voltage, param);
    } else {
        norm_thrust = des_acc_norm / thr2acc_;
    }

    return norm_thrust;
}

double ControllerClass::AccurateThrustAccMapping(const double des_acc_z, double voltage, const Param &param)
{
    // F = K1*Voltage^K2*(K3*u^2+(1-K3)*u)
    double a = param.thr_model_.K3;
    double b = 1 - param.thr_model_.K3;
    double c = -(param.mass_ * des_acc_z) / (param.thr_model_.K1 * std::pow(voltage, param.thr_model_.K2));
    double b2_4ac = pow(b, 2) - 4 * a * c;
    if (b2_4ac <= 0) b2_4ac = 0;
    double thrust = (-b + sqrt(b2_4ac)) / (2 * a);
    if (thrust <= 0) thrust = 0; // This should be avoided before calling this function
    return thrust;
}

bool ControllerClass::estimateThrustModel(const Eigen::Vector3d &est_a, const double voltage, const Param &param)
{
    ros::Time t_now = ros::Time::now();
    if (timed_thrust_.size() == 0) return false;
    std::pair<ros::Time, double> t_t = timed_thrust_.front();

    while (timed_thrust_.size() >= 1) {
        double delta_t = (t_now - t_t.first).toSec();
        if (delta_t > 1.0) {
            timed_thrust_.pop();
            continue;
        } 
        if (delta_t < 0.035) {
            return false;
        }

        /* Recursive least squares algorithm with vanishing memory */
        double thr = t_t.second;
        timed_thrust_.pop();
        if (param.thr_model_.accurate_thrust_model_flag) {

        } else {
            /* Model: est_a(2) = thr2acc * thr */
            double R = 0.3; // using Kalman filter
            double K = P_ / (P_ + R);
            thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
            P_ = (1 - K * thr) * P_;
            // double gamma = 1 / (rho2_ + thr * P_ * thr);
            // double K = gamma * P_ * thr;
            // thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
            // P_ = (1 - K * thr) * P_ / rho2_;
            double hover_percentage = param.gra_ / thr2acc_;
            if (hover_percentage > 0.8 || hover_percentage < 0.1) {
                ROS_INFO_THROTTLE(1, "Estimated hover_percentage >0.8 or <0.1! Perhaps the accel vibration is too high!");
                thr2acc_ = hover_percentage > 0.8 ? param.gra_ / 0.8 : thr2acc_;
                thr2acc_ = hover_percentage < 0.1 ? param.gra_ / 0.1 : thr2acc_;
            }
            if (param.thr_model_.print_flag) {
                ROS_WARN("[PX4CTRL] hover_percentage = %f", hover_percentage);
            }
        }
        return true;
    }
    return false;
}
