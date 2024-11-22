#include "poly_planning.h"

PolyplanningClass::PolyplanningClass(ros::NodeHandle &nh) : nh_(nh)
{
    int order;
    double v_max, a_max;
    nh_.param("/planning_node/order", order, 3);
    nh_.param("/planning_node/v_max", v_max, 3.0);
    nh_.param("/planning_node/a_max", a_max, 1.0);
    yaw_ = 0;
    yaw_dot_ = 0;
    poly_traj_ = std::make_shared<PolyTrajClass>(order, v_max, a_max);
    waypoints_list.clear();
    last_waypoint_num = -1;
    cmd_p_(0) = 0 ;
    cmd_p_(1) = 0;
    cmd_p_(2) = -1.5;
    yaw_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("yaw_ctrl", 1, &PolyplanningClass::YawCtrlCallback, this, ros::TransportHints().tcpNoDelay());
    odom_sub_  = nh_.subscribe<nav_msgs::Odometry>("/airsim_bridge/drone_1/debug/odom", 1, &PolyplanningClass::OdomCallback, this, ros::TransportHints().tcpNoDelay());
    rviz_points_sub_ = nh_.subscribe<nav_msgs::Path>("waypoints", 1, &PolyplanningClass::WaypointsCallback, this, ros::TransportHints().tcpNoDelay());

    trajectory_pub_ = nh_.advertise<nav_msgs::Path>("path", 1);
    cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("pos_cmd", 1);

    timer_ = nh_.createTimer(ros::Duration(1.0/100.0), &PolyplanningClass::TimerCallback, this, false, true);
}

void PolyplanningClass::TimerCallback(const ros::TimerEvent &)
{
    if (time_.size() == 0) {
        Eigen::Vector3d v_r = Eigen::Vector3d::Zero();
        CmdPublish(Eigen::Vector3d(0, 0, -2), v_r, v_r, v_r);
        return;
    }

    if (traj_flag_) {
        t_start_ = ros::Time::now();
        traj_flag_ = false;
    }

    double t_delta = (ros::Time::now() - t_start_).toSec();
    Eigen::Vector3d pos  = poly_traj_->Getposition(t_delta);
    Eigen::Vector3d vel  = poly_traj_->GetVelocity(t_delta);
    Eigen::Vector3d acc  = poly_traj_->GetAcceleration(t_delta);
    Eigen::Vector3d jerk = poly_traj_->GetJerk(t_delta);
    yaw_ = getYawPoly(vel(0),vel(1));
    std::cout<<"yaw:"<<yaw_<<"velx:"<<vel(0)<<"vely"<<vel(1)<<std::endl;
    CmdPublish(pos, vel, acc, jerk);
}

void PolyplanningClass::OdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    odom_mutex_.lock();
    odom_p_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_v_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z; 
    odom_mutex_.unlock();
}

void PolyplanningClass::YawCtrlCallback(const geometry_msgs::TwistStampedConstPtr& msg)
{
    yaw_mutex_.lock();
    yaw_ = msg->twist.linear.z;
    yaw_dot_ = msg->twist.angular.z;
    yaw_mutex_.unlock();
}

void PolyplanningClass::TrajectoryPub(ros::Time stamp)
{
    nav_msgs::Path trajectory;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = stamp;

    geometry_msgs::PoseStamped pose_stamped;
    double t_all = poly_traj_->TotleTime();
    for (double t = 0.0; t < t_all; t += 0.01) {
        Eigen::Vector3d pos = poly_traj_->Getposition(t);
        pose_stamped.pose.position.x = pos.x();
        pose_stamped.pose.position.y = pos.y();
        pose_stamped.pose.position.z = pos.z();
        trajectory.poses.emplace_back(pose_stamped);
    }

    trajectory_pub_.publish(trajectory);
}

void PolyplanningClass::WaypointsCallback(const nav_msgs::PathConstPtr& msg)
{
    waypoints_mutex_.lock();
    int waypoints_num = msg->poses.size();
    if (waypoints_num == 1) {
        ROS_WARN("Please check your waypoints!");
        return;
    }
    //int msg_point_num = waypoints_num+1;
    // if(last_waypoint_num == -1|| last_waypoint_num!=msg_point_num)
    // {
        
    //     int path_size = msg_point_num+1;
    //     // Eigen::Vector3d temp_waypoint;
    //     // waypoints_list.clear();
        //waypoints_list.resize(path_size);
        Eigen::Vector3d first_point(msg->poses[0].pose.position.x,
                                    msg->poses[0].pose.position.y,
                                    msg->poses[0].pose.position.z);
        if((cmd_p_-first_point).norm()<0.1)
        {
            //NO NEED TO ADD CURRENT POINT
            Eigen::MatrixXd waypoints = Eigen::MatrixXd::Zero(waypoints_num, 3);
            for (int i = 1; i < waypoints_num+1; i++) {
            waypoints(i, 0) = msg->poses[i].pose.position.x;
            waypoints(i, 1) = msg->poses[i].pose.position.y;
            waypoints(i, 2) = msg->poses[i].pose.position.z;
            }
            t_start_ = ros::Time::now();
            poly_traj_->AllocateTime(waypoints, time_);
            poly_traj_->TrajGenerate(waypoints, cmd_v_, Eigen::Vector3d::Zero(), cmd_a_, Eigen::Vector3d::Zero(), time_);
            traj_flag_ = true;
        }
        else
        {
            Eigen::MatrixXd waypoints = Eigen::MatrixXd::Zero(waypoints_num+1, 3);
            waypoints(0,0)= odom_p_.x();
            waypoints(0,1)= odom_p_.y();
            waypoints(0,2)= odom_p_.z();
            for (int i = 1; i < waypoints_num+1; i++) {
                waypoints(i, 0) = msg->poses[i-1].pose.position.x;
                waypoints(i, 1) = msg->poses[i-1].pose.position.y;
                waypoints(i, 2) = msg->poses[i-1].pose.position.z;
            }
            t_start_ = ros::Time::now();
            poly_traj_->AllocateTime(waypoints, time_);
            poly_traj_->TrajGenerate(waypoints, cmd_v_, Eigen::Vector3d::Zero(), cmd_a_, Eigen::Vector3d::Zero(), time_);
            traj_flag_ = true;
        }
    std::cout<<"LAST WAYPOINTS: "<<std::endl;
  
    ROS_INFO("\033[1;32mTime in Minimum Snap is %f ms\033[0m", (ros::Time::now() - t_start_).toSec() * 1000.0);
    
    TrajectoryPub(t_start_); // just for visualization

    waypoints_mutex_.unlock();
}
