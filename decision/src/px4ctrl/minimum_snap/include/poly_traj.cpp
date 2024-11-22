#include "poly_traj.h"

static int Factorial(int x) {
    int fac = 1;
    for (int i = x; i > 0; --i) {
        fac = fac * i;
    }
    return fac;
}

void PolyTrajClass::AllocateTime(const Eigen::MatrixXd &waypoints, Eigen::VectorXd &time)
{
    /* Trapezoidal velocity planning */
    time = Eigen::VectorXd::Zero(waypoints.rows() - 1);
    double dis_threshold = v_max_ * v_max_ / a_max_;
    double t_segment;
    for (unsigned int i = 0; i < time.rows(); i++) {
        double dis = (waypoints.row(i+1) - waypoints.row(i)).norm();
        if (dis > dis_threshold) {
            t_segment = v_max_ / a_max_ * 2 + (dis - dis_threshold) / v_max_;
        } else {
            t_segment = std::sqrt(dis / a_max_);
        }
        time[i] = t_segment;
    }
}

void PolyTrajClass::TrajGenerate(const Eigen::MatrixXd &pos, const Eigen::Vector3d &start_vel,
                                 const Eigen::Vector3d &end_vel, const Eigen::Vector3d &start_acc,
                                 const Eigen::Vector3d &end_acc, const Eigen::VectorXd &time)
{
    const unsigned int poly_order = 2 * order_ - 1;
    const unsigned int num_poly_coeff = poly_order + 1;
    const unsigned int num_segments = time.size();
    const unsigned int num_all_poly_coeff = num_poly_coeff * num_segments;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(num_all_poly_coeff, num_all_poly_coeff);
    for (int m = 0; m < num_segments; m++) {
        Eigen::MatrixXd Q_sub = Eigen::MatrixXd::Zero(num_poly_coeff, num_poly_coeff);
        for (int i = order_; i <= poly_order; i++) {
            for (int j = order_; j <= poly_order; j++) {
                Q_sub(i, j) = (Factorial(i) / Factorial(i - order_)) * (Factorial(j) / Factorial(j - order_)) / 
                              (i + j - poly_order) * std::pow(time[m], i + j - poly_order);
            }
        }
        Q.block(m * num_poly_coeff, m * num_poly_coeff, num_poly_coeff, num_poly_coeff) = Q_sub;
    }

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_all_poly_coeff, num_all_poly_coeff);
    for (int m = 0; m < num_segments; m++) {
        Eigen::MatrixXd A_sub = Eigen::MatrixXd::Zero(num_poly_coeff, num_poly_coeff);
        for (int i = 0; i < order_; i++) {
            for (int j = i; j < num_poly_coeff; j++) {
                A_sub(i, j) = Factorial(j) / Factorial(j - i) * std::pow(0, j - i);
                A_sub(i + order_, j) = Factorial(j) / Factorial(j - i) * std::pow(time[m], j - i);
            }
        }
        A.block(m * num_poly_coeff, m * num_poly_coeff, num_poly_coeff, num_poly_coeff) = A_sub;
    }

    const unsigned int num_fixed_variables = 2u * order_ + (num_segments - 1);
    Eigen::MatrixXd C_T = Eigen::MatrixXd::Zero(num_all_poly_coeff, (num_segments + 1) * order_);
    for (unsigned int i = 0; i < num_all_poly_coeff; i++) {
        if (i < order_) {
            C_T(i, i) = 1.0;
            continue;
        }
        if (i >= num_all_poly_coeff - order_) {
            const unsigned int delta_index = i - (num_all_poly_coeff - order_);
            C_T(i, num_fixed_variables - order_ + delta_index) = 1.0;
            continue;
        }
        if ((i % order_ == 0u) && (i / order_ % 2u == 1u)) {
            const unsigned int index = i / (2u * order_) + order_;
            C_T(i, index) = 1.0;
            continue;
        }
        if ((i % order_ == 0u) && (i / order_ % 2u == 0u)) {
            const unsigned int index = i / (2u * order_) + order_ - 1u;
            C_T(i, index) = 1.0;
            continue;
        }
        if ((i % order_ != 0u) && (i / order_ % 2u == 1u)) {
            const unsigned int temp_index_0 = i / (2 * order_) * (2 * order_) + order_;
            const unsigned int temp_index_1 = i / (2 * order_) * (order_ - 1) + i - temp_index_0 - 1;
            C_T(i, num_fixed_variables + temp_index_1) = 1.0;
            continue;
        }
        if ((i % order_ != 0u) && (i / order_ % 2u == 0u)) {
            const unsigned int temp_index_0 = (i - order_) / (2 * order_) * (2 * order_) + order_;
            const unsigned int temp_index_1 = (i - order_) / (2 * order_) * (order_ - 1) + (i - order_) - temp_index_0 - 1;
            C_T(i, num_fixed_variables + temp_index_1) = 1.0;
            continue;
        }
    }

    Eigen::MatrixXd R = C_T.transpose() * A.transpose().inverse() * Q * A.inverse() * C_T;
    Eigen::MatrixXd R_PP = R.block(num_fixed_variables, num_fixed_variables, 
                                   (num_segments + 1) * order_ - num_fixed_variables, 
                                   (num_segments + 1) * order_ - num_fixed_variables);
    Eigen::MatrixXd R_PF = R.block(num_fixed_variables, 0, (num_segments + 1) * order_ - num_fixed_variables, num_fixed_variables);

    Eigen::VectorXd dx_FP = Eigen::VectorXd::Zero((num_segments + 1) * order_);
    Eigen::VectorXd dy_FP = Eigen::VectorXd::Zero((num_segments + 1) * order_);
    Eigen::VectorXd dz_FP = Eigen::VectorXd::Zero((num_segments + 1) * order_);
    
    dx_FP[0] = pos(0, 0);
    dx_FP[num_fixed_variables - order_] = pos(num_segments, 0);
    dx_FP[1] = start_vel(0);
    dx_FP[num_fixed_variables - order_ + 1] = end_vel(0);
    dx_FP[2] = start_acc(0);
    dx_FP[num_fixed_variables - order_ + 2] = end_acc(0);

    dy_FP[0] = pos(0, 1);
    dy_FP[num_fixed_variables - order_] = pos(num_segments, 1);
    dy_FP[1] = start_vel(1);
    dy_FP[num_fixed_variables - order_ + 1] = end_vel(1);
    dy_FP[2] = start_acc(1);
    dy_FP[num_fixed_variables - order_ + 2] = end_acc(1);

    dz_FP[0] = pos(0, 2);
    dz_FP[num_fixed_variables - order_] = pos(num_segments, 2);
    dz_FP[1] = start_vel(2);
    dz_FP[num_fixed_variables - order_ + 1] = end_vel(2);
    dz_FP[2] = start_acc(2);
    dz_FP[num_fixed_variables - order_ + 2] = end_acc(2);
    
    for (int i = 0; i < num_segments - 1; i++) {
        dx_FP[order_ + i] = pos(i + 1, 0);
        dy_FP[order_ + i] = pos(i + 1, 1);
        dz_FP[order_ + i] = pos(i + 1, 2);
    } 

    Eigen::VectorXd dx_F = dx_FP.head(num_fixed_variables);
    Eigen::VectorXd dx_P = -R_PP.inverse() * R_PF * dx_F;
    dx_FP.tail((num_segments + 1) * order_ - num_fixed_variables) = dx_P;
    Eigen::VectorXd Px = A.inverse() * C_T * dx_FP;

    Eigen::VectorXd dy_F = dy_FP.head(num_fixed_variables);
    Eigen::VectorXd dy_P = -R_PP.inverse() * R_PF * dy_F;
    dy_FP.tail((num_segments + 1) * order_ - num_fixed_variables) = dy_P;
    Eigen::VectorXd Py = A.inverse() * C_T * dy_FP;

    Eigen::VectorXd dz_F = dz_FP.head(num_fixed_variables);
    Eigen::VectorXd dz_P = -R_PP.inverse() * R_PF * dz_F;
    dz_FP.tail((num_segments + 1) * order_ - num_fixed_variables) = dz_P;
    Eigen::VectorXd Pz = A.inverse() * C_T * dz_FP;

    poly_coeff_mat_ = Eigen::MatrixXd::Zero(num_segments, num_poly_coeff * 3); // Polynomial coefficient: Vector to Matrix
    time_ = Eigen::VectorXd::Zero(num_segments);
    for (unsigned int i = 0; i < num_segments; i++) {
        poly_coeff_mat_.block(i, 0, 1, num_poly_coeff)  = Px.block(num_poly_coeff * i, 0, num_poly_coeff, 1).transpose();
        poly_coeff_mat_.block(i, num_poly_coeff, 1, num_poly_coeff)  = Py.block(num_poly_coeff * i, 0, num_poly_coeff, 1).transpose();
        poly_coeff_mat_.block(i, 2*num_poly_coeff, 1, num_poly_coeff) = Pz.block(num_poly_coeff * i, 0, num_poly_coeff, 1).transpose();
        time_(i) = time(i);
    }

}