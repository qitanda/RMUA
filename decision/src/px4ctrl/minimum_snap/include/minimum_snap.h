#ifndef MINIMUM_SNAP_H
#define MINIMUM_SNAP_H

#include <ros/ros.h>
#include <Eigen/Eigen>

typedef typename Eigen::MatrixXd MatXd;
typedef typename Eigen::VectorXd VecXd;
typedef typename Eigen::Vector2d Vec2d;
typedef typename Eigen::Vector3d Vec3d;

class MinimumSnapClass {
public:
    MinimumSnapClass(unsigned int order, double v_max, double acc_max) : 
                    order_(order), v_max_(v_max), acc_max_(acc_max) {}
    ~MinimumSnapClass(){}

    int GetPolyCoeffNum() {
        return 2 * order_;
    }

    void AllocateTime(const MatXd &waypoints, VecXd &time);
    void SolveQPCloseForm(const VecXd &pos, const Vec2d &vel, const Vec2d &acc,
                           const VecXd &time, MatXd &poly_coeff_mat);

private:
    const unsigned int order_;
    const double v_max_;
    const double acc_max_;
};

#endif
