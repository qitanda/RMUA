#include "minimum_snap.h"

inline double Factorial(unsigned int x);

void MinimumSnapClass::AllocateTime(const MatXd &waypoints, VecXd &time)
{
    /* Trapezoidal velocity planning */
    time = VecXd::Zero(waypoints.rows() - 1);
    double dis_threshold = v_max_ * v_max_ / acc_max_;
    double t_segment;
    for (unsigned int i = 0; i < time.rows(); i++) {
        double dis = (waypoints.row(i+1) - waypoints.row(i)).norm();
        if (dis > dis_threshold) {
            t_segment = v_max_ / acc_max_ + (dis - dis_threshold) / v_max_;
        } else {
            t_segment = std::sqrt(dis / acc_max_);
        }
        time[i] = t_segment;
    }
}

void MinimumSnapClass::SolveQPCloseForm(const VecXd &pos, const Vec2d &vel, const Vec2d &acc, 
                                        const VecXd &time, MatXd &poly_coeff_mat)
{
    const unsigned int poly_order = 2 * order_ - 1;
    const unsigned int num_poly_coeff = poly_order + 1;
    const unsigned int num_segments = time.size();
    const unsigned int num_all_poly_coeff = num_poly_coeff * num_segments;

    MatXd Q = MatXd::Zero(num_all_poly_coeff, num_all_poly_coeff);
    for (int m = 0; m < num_segments; m++) {
        MatXd Q_sub = MatXd::Zero(num_poly_coeff, num_poly_coeff);
        for (int i = order_; i <= poly_order; i++) {
            for (int j = order_; j <= poly_order; j++) {
                // if (i < order_ || j < order_) {
                //     continue;
                // }
                Q_sub(i, j) = (Factorial(i) / Factorial(i - order_)) * (Factorial(j) / Factorial(j - order_)) / 
                              (i + j - poly_order) * std::pow(time[m], i + j - poly_order);
            }
        }
        Q.block(m * num_poly_coeff, m * num_poly_coeff, num_poly_coeff, num_poly_coeff) = Q_sub;
    }
    // std::cout << "Q:" << std::endl << Q << std::endl;

    MatXd A = MatXd::Zero(num_all_poly_coeff, num_all_poly_coeff);
    for (int m = 0; m < num_segments; m++) {
        MatXd A_sub = MatXd::Zero(num_poly_coeff, num_poly_coeff);
        for (int i = 0; i < order_; i++) {
            for (int j = i; j < num_poly_coeff; j++) {
                // if (i > j) {
                //     continue;
                // }
                A_sub(i, j) = Factorial(j) / Factorial(j - i) * std::pow(0, j - i);
                A_sub(i + order_, j) = Factorial(j) / Factorial(j - i) * std::pow(time[m], j - i);
            }
        }
        A.block(m * num_poly_coeff, m * num_poly_coeff, num_poly_coeff, num_poly_coeff) = A_sub;
    }
    // std::cout << "A:" << std::endl << A << std::endl;

    const unsigned int num_fixed_variables = 2u * order_ + (num_segments - 1);
    MatXd C_T = MatXd::Zero(num_all_poly_coeff, (num_segments + 1) * order_);
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
    // std::cout << "C_T:" << std::endl << C_T << std::endl;

    MatXd R = C_T.transpose() * A.transpose().inverse() * Q * A.inverse() * C_T;
    VecXd d_FP = VecXd::Zero((num_segments + 1) * order_);
    d_FP[0] = pos[0];
    d_FP[num_fixed_variables - order_] = pos[num_segments];
    if (order_ >= 2) {
        d_FP[1] = vel[0];
        d_FP[num_fixed_variables - order_ + 1] = vel[1];
    }
    if (order_ >= 3) {
        d_FP[2] = acc[0];
        d_FP[num_fixed_variables - order_ + 2] = acc[1];
    }
    for (int i = 0; i < num_segments - 1; i++) {
        d_FP[order_ + i] = pos[i + 1];
    } 
    // std::cout << "d_FP:" << std::endl << d_FP.transpose() << std::endl;

    VecXd d_F = d_FP.head(num_fixed_variables);
    MatXd R_PP = R.block(num_fixed_variables, num_fixed_variables, 
                        (num_segments + 1) * order_ - num_fixed_variables, 
                        (num_segments + 1) * order_ - num_fixed_variables);
    MatXd R_PF = R.block(num_fixed_variables, 0, (num_segments + 1) * order_ - num_fixed_variables, num_fixed_variables);
    VecXd d_P = -R_PP.inverse() * R_PF * d_F;
    d_FP.tail((num_segments + 1) * order_ - num_fixed_variables) = d_P;

    VecXd P = A.inverse() * C_T * d_FP;

    poly_coeff_mat = MatXd::Zero(num_segments, num_poly_coeff); // Polynomial coefficient: Vector to Matrix
    for (unsigned int i = 0; i < num_segments; i++) {
        poly_coeff_mat.block(i, 0, 1, num_poly_coeff) = P.block(num_poly_coeff * i, 0, num_poly_coeff, 1).transpose();
    }
    // std::cout << "poly_coeff_mat:" << std::endl << poly_coeff_mat << std::endl;
    // return poly_coeff_mat;
}

inline double Factorial(unsigned int x) {
    unsigned int fac = 1;
    for (unsigned int i = x; i > 0; --i) {
        fac = fac * i;
    }
    return static_cast<double>(fac);
}