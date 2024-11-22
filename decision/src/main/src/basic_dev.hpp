#ifndef _BASIC_DEV_HPP_
#define _BASIC_DEV_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "airsim_ros/VelCmd.h"
#include "airsim_ros/PoseCmd.h"
#include "airsim_ros/Takeoff.h"
#include "airsim_ros/Reset.h"
#include "airsim_ros/Land.h"
#include "airsim_ros/GPSYaw.h"
#include "airsim_ros/CirclePoses.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include <time.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <ros/callback_queue.h>
#include <boost/thread/thread.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "airsim_ros/AngleRateThrottle.h"
#include <nav_msgs/Path.h>
#endif

class BasicDev
{
private:
    std::unique_ptr<image_transport::ImageTransport> it;
    ros::CallbackQueue go_queue;
    ros::CallbackQueue front_img_queue;

    // 调用服务前需要定义特定的调用参数
    airsim_ros::Takeoff takeoff;
    airsim_ros::Land land;
    airsim_ros::Reset reset;

    // 使用publisher发布速度指令需要定义 Velcmd , 并赋予相应的值后，将他publish（）出去
    airsim_ros::VelCmd velcmd;

    // 使用publisher发布姿态指令需要定义 Posecmd , 并赋予相应的值后，将他publish（）出去
    // airsim_ros::PoseCmd posecmd;

    // 使用publisher发布角速度指令需要定义 AngleRateThrottle , 并赋予相应的值后，将他publish（）出去
    airsim_ros::AngleRateThrottle arthrcmd;

    // 无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    ros::Subscriber odom_suber;    // 状态真值
    ros::Subscriber gps_suber;     // gps数据
    ros::Subscriber imu_suber;     // imu数据
    ros::Subscriber circles_suber; // 障碍圈参考位置
    image_transport::Subscriber front_left_view_suber;
    image_transport::Subscriber front_right_view_suber;
    image_transport::Subscriber bottom_view_suber;

    ros::Subscriber img_circle_suber;
    ros::Subscriber gt_circle_suber;

    // 通过这两个服务可以调用模拟器中的无人机起飞和降落命令
    ros::ServiceClient takeoff_client;
    ros::ServiceClient land_client;
    ros::ServiceClient reset_client;

    // 通过这两个publisher实现对无人机的速度控制和姿态控制
    ros::Publisher vel_publisher;
    ros::Publisher pose_publisher;
    ros::Publisher anglerate_publisher;
    ros::Publisher path_pub;
    ros::Publisher mode_pub;
    ros::Publisher count_pub;
    ros::Publisher yaw_pub;

    void gt_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void gps_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void imu_cb(const sensor_msgs::Imu::ConstPtr &msg);
    void origin_circles_cb(const airsim_ros::CirclePoses::ConstPtr &msg);
    void img_circle_cb(const airsim_ros::Circle::ConstPtr &msg);
    void gt_circle_cb(const airsim_ros::CirclePoses::ConstPtr &msg);

    void bottom_view_cb(const sensor_msgs::ImageConstPtr &msg);
    void front_left_view_cb(const sensor_msgs::ImageConstPtr &msg);
    void front_right_view_cb(const sensor_msgs::ImageConstPtr &msg);

    nav_msgs::Path get_1to3_path(const airsim_ros::Circle circle_position);
    geometry_msgs::PoseStamped get_midpoint(const nav_msgs::Path fron_path, const nav_msgs::Path behi_path);
    nav_msgs::Path get_path(airsim_ros::Circle made_path_circle_0, airsim_ros::Circle made_path_circle_1);
    bool JudgeCircleThroughState();
    bool JudgePointThroughState();
    void YawPublish();

    //Debug message
    int last_check_index;

public:
    BasicDev(ros::NodeHandle *nh);
    ~BasicDev();
};
