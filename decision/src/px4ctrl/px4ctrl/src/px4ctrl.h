#ifndef PX4CTRL_H
#define PX4CTRL_H

#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Eigen>

#include "../include/param.h"
#include "../include/blackboard.h"
#include "../include/controller.h"


#include "airsim_ros/VelCmd.h"
#include "airsim_ros/PoseCmd.h"
#include "airsim_ros/Takeoff.h"
#include "airsim_ros/Reset.h"
#include "airsim_ros/Land.h"
#include "airsim_ros/GPSYaw.h"
#include <geometry_msgs/PoseStamped.h>

class PX4CrtlNode{
public:
    PX4CrtlNode(ros::NodeHandle &nh);
    ~PX4CrtlNode(){}

private:
    void CtrlTimerCallback(const ros::TimerEvent &);
    // void UpdateFSM(ros::Time &t_now, DesireState_t &des);
    // DesireState_t GetHoverDes(ros::Time &t_now);
    DesireState_t GetCmdDes(void);
    void AttitudeCtrlPub(const CtrlOutput_t &u, const ros::Time &stamp);
    void BodyrateCtrlPub(const CtrlOutput_t &u, const ros::Time &stamp);

    ros::NodeHandle nh_;
    ros::Timer ctrl_timer_;
    ros::Publisher px4ctrl_pub_, airsim_pub_;
    ros::ServiceClient arming_client_, set_mode_client_;
    //通过这两个服务可以调用模拟器中的无人机起飞和降落命令
    ros::ServiceClient takeoff_client;
    ros::ServiceClient land_client;
    ros::ServiceClient reset_client;
    // 调用服务前需要定义特定的调用参数
    airsim_ros::Takeoff takeoff;
    airsim_ros::Land land;
    airsim_ros::Reset reset;
    Param param_;
    Blackboard bd_;
    ControllerClass ctrl_;
    CtrlOutput_t u;
    DesireState_t hover_des_;
    bool hover_z_flag_;
    double takeoff_height_start_, height_last_;
    ros::Time height_last_time_;
    int height_land_num_;
    bool odom_start_init = false;
    bool odom_init_ready = false;
    bool wait_cmd_falg_ = false;
};

#endif
