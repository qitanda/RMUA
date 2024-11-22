#ifndef POLY_PLANNING_H_
#define POLY_PLANNING_H_

#include <ros/ros.h>
#include <mutex>

#include "../include/poly_traj.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include "quadrotor_msgs/PositionCommand.h"


class PolyplanningClass {
public:
    PolyplanningClass(ros::NodeHandle &nh);
    ~PolyplanningClass(){}

private:
    void TrajectoryPub(ros::Time stamp);
    void WaypointsCallback(const nav_msgs::PathConstPtr& msg);
    void TimerCallback(const ros::TimerEvent &);
    void OdomCallback(const nav_msgs::OdometryConstPtr& msg);
    void YawCtrlCallback(const geometry_msgs::TwistStampedConstPtr& msg);

    void CmdPublish(Eigen::Vector3d p_r, Eigen::Vector3d v_r, Eigen::Vector3d a_r, Eigen::Vector3d j_r) {
        quadrotor_msgs::PositionCommand msg;
        msg.header.frame_id = "world";
        msg.header.stamp    = ros::Time::now();
        msg.position.x      = p_r.x();
        msg.position.y      = p_r.y();
        msg.position.z      = p_r.z();
        msg.velocity.x      = v_r.x();
        msg.velocity.y      = v_r.y();
        msg.velocity.z      = v_r.z();
        msg.acceleration.x  = a_r.x();
        msg.acceleration.y  = a_r.y();
        msg.acceleration.z  = a_r.z();
        msg.jerk.x          = j_r.x();
        msg.jerk.y          = j_r.y();
        msg.jerk.z          = j_r.z();
        msg.yaw             = yaw_;
        msg.yaw_dot         = yaw_dot_;
        cmd_pub_.publish(msg);
    }    

    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Subscriber rviz_points_sub_, odom_sub_, yaw_sub_;
    ros::Publisher trajectory_pub_, cmd_pub_;
    ros::Time t_start_;

    bool traj_flag_;
    double yaw_, yaw_dot_;

    std::shared_ptr<PolyTrajClass> poly_traj_;
    std::mutex waypoints_mutex_, odom_mutex_, yaw_mutex_;

    Eigen::Vector3d odom_p_, odom_v_;
    
    Eigen::VectorXd time_;
};

#endif
