#ifndef Blackboard_H
#define Blackboard_H

#include <ros/ros.h>
#include <mutex>

#include <Eigen/Eigen>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandLong.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
enum UAVMode_e {
    Manual  = 0,
    Hover   = 1,
    Takeoff = 2,
    Land    = 3,
    Command = 4
};
class OdomClass{
public:
    OdomClass(){}
    ~OdomClass(){}

    void Init(bool simulate_flag) {
        simulate_flag_ = simulate_flag;
        stamp_ = ros::Time::now();
    }
    void Callback(const nav_msgs::Odometry::ConstPtr &msg) {
        mutex_.lock();
        stamp_ = ros::Time::now();
        if (simulate_flag_ == false) {
            p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
            v << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
            w << msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;
            q = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
           
                                
        } else {
            q = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                   msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
            Eigen::Matrix3d wRb = q.normalized().toRotationMatrix();
            p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
            v << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
            // v = wRb * v;
            double roll,pitch,yaw;
            tf2::Quaternion tf_q(q.x(),q.y(),q.z(),q.w());
            tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);// 四元数转欧拉角
            ROS_INFO("BLACKBOARD!!!roll=%.2f, pitch=%.2f, yaw=%.2f", roll, pitch, yaw);
            ROS_INFO("BLACKBOARD!!!x=%.2f, y=%.2f, z=%.2f", p.x(), p.y(), p.z());
            w << msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;
        }
        // std::cout << "qqq" << std::endl;
        mutex_.unlock();
    }
    
    Eigen::Vector3d p, v, w;
    Eigen::Quaterniond q;

    ros::Time stamp_;
    std::mutex mutex_;
    bool simulate_flag_;
};

class StateClass{
public:
    StateClass(){}
    ~StateClass(){}

    void Init(void) {
        stamp_ = ros::Time::now();
    }
    void Callback(const mavros_msgs::State::ConstPtr &msg) {
        mutex_.lock();
        stamp_ = ros::Time::now();
        state = *msg;
        // std::cout << "UAV state:" << state.mode << std::endl;
        mutex_.unlock();
    }
    
    mavros_msgs::State state;

    ros::Time stamp_;
    std::mutex mutex_;
};

class IMUDateClass{
public:
    IMUDateClass(){}
    ~IMUDateClass(){}

    void Init(void) {
        stamp_ = ros::Time::now();
    }
    void Callback(const sensor_msgs::Imu::ConstPtr &msg) {
        mutex_.lock();
        stamp_ = ros::Time::now();
        w << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
        a << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        q = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, 
                               msg->orientation.y, msg->orientation.z);
        // std::cout << "imu: " << a.x() << " " << a.y() << " " << a.z() << std::endl;
        mutex_.unlock();
    }
    
    Eigen::Quaterniond q;
    Eigen::Vector3d w;
    Eigen::Vector3d a;

    ros::Time stamp_;
    std::mutex mutex_;
};



class BatteryClass{
public:
    BatteryClass(){}
    ~BatteryClass(){}

    void Init(void) {
        stamp_ = ros::Time::now();
    }
    void Callback(const sensor_msgs::BatteryState::ConstPtr &msg) {
        mutex_.lock();
        stamp_ = ros::Time::now();
        double voltage = 0;
        for (size_t i = 0; i < msg->cell_voltage.size(); ++i) {
            voltage += msg->cell_voltage[i];
        }
        volt = 0.8 * volt + 0.2 * voltage;
        current = 0.8 * current + msg->current * 0.2;
        percentage = msg->percentage;
        // std::cout << "battery: " << volt << " " << current << " " << percentage << std::endl;
        mutex_.unlock();
    }
    
    double volt{0.0};
    double current{0.0};
    double percentage{0.0};

    ros::Time stamp_;
    std::mutex mutex_;
};

class CmdClass{
public:
    CmdClass(){}
    ~CmdClass(){}

    void Init(void) {
        stamp_ = ros::Time::now();
        p = Eigen::Vector3d(0, 0, 0);
        v = Eigen::Vector3d(0, 0, 0);
        a = Eigen::Vector3d(0, 0, 0);
        j = Eigen::Vector3d(0, 0, 0);
        yaw = 0;
        yaw_rate = 0;
        debug_cmd_flag = false;
    }
    Eigen::Vector3d CheckNan(const Eigen::Vector3d vec) {
        Eigen::Vector3d result;
        result = vec;
        if (std::isnan(vec.x())) result.x() = 0;
        if (std::isnan(vec.y())) result.y() = 0;
        if (std::isnan(vec.z())) result.z() = 0;
        return result;
    }
    void Callback(const quadrotor_msgs::PositionCommand::ConstPtr &msg) {
        if(!debug_cmd_flag) debug_cmd_flag = true;
        mutex_.lock();
        stamp_ = ros::Time::now();
        p = CheckNan(Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z));
        v = CheckNan(Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z));
        a = CheckNan(Eigen::Vector3d(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z));
        j = CheckNan(Eigen::Vector3d(msg->jerk.x, msg->jerk.y, msg->jerk.z));
        yaw = msg->yaw; // norm: -PI~PI
        while (true) {
            if(yaw < -M_PI) yaw += M_PI * 2.0;
            else if(yaw > M_PI) yaw -= M_PI * 2.0;
            else break;
        }
        yaw_rate = msg->yaw_dot;
        // std::cout << "cmd: " << p.x() << " " << p.y() << " " << p.z() << std::endl;
        mutex_.unlock();
    }
    
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    double yaw;
    double yaw_rate;
    bool debug_cmd_flag;
    ros::Time stamp_;
    std::mutex mutex_;
};

class Blackboard{
public:
    Blackboard(){}
    ~Blackboard(){}

    void Init(ros::NodeHandle &nh){
        nh.param("/px4ctrl_node/airsim_flag", airsim_flag_, true);
        nh.param("/px4ctrl_node/gazebo_flag", gazebo_flag_, false);
        nh.param("/px4ctrl_node/rc_hover_gain", rc_hover_gain_, 0.1);

        odom_.Init(airsim_flag_);
        odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&OdomClass::Callback, &odom_, _1), 
                                                    ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
        // state_.Init();
        // state_sub_ = nh.subscribe<mavros_msgs::State>("state", 1, boost::bind(&StateClass::Callback, &state_, _1), 
        //                                             ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
        imu_.Init();
        imu_sub_ = nh.subscribe<sensor_msgs::Imu>("imu", 1, boost::bind(&IMUDateClass::Callback, &imu_, _1), 
                                                    ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
        // rc_.Init(&reboot_srv_, &mode_);
        // rc_sub_ = nh.subscribe<mavros_msgs::RCIn>("rc_in", 1, boost::bind(&RCInClass::Callback, &rc_, _1), 
        //                                             ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
        battery_.Init();
        battery_sub_ = nh.subscribe<sensor_msgs::BatteryState>("battery", 1, boost::bind(&BatteryClass::Callback, &battery_, _1), 
                                                    ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
        cmd_.Init();
        cmd_sub_ = nh.subscribe<quadrotor_msgs::PositionCommand>("pos_cmd", 1, boost::bind(&CmdClass::Callback, &cmd_, _1), 
                                                    ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    
        // reboot_srv_ = nh.serviceClient<mavros_msgs::CommandLong>("reboot");
    }

    bool airsim_flag_, gazebo_flag_;
    double rc_hover_gain_;
    UAVMode_e mode_;
    OdomClass odom_;
    // StateClass state_;
    IMUDateClass imu_;
    // RCInClass rc_;
    BatteryClass battery_;
    CmdClass cmd_;

private:
    ros::Subscriber odom_sub_, imu_sub_, battery_sub_, cmd_sub_;
    // ros::Subscriber state_sub_,rc_sub_;
    // ros::ServiceClient reboot_srv_;
};

#endif