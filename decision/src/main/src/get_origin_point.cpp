#ifndef _GET_ORIGIN_POINT_CPP_
#define _GET_ORIGIN_POINT_CPP_

#include "basic_dev.hpp"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <flag_msgs/count_circle.h>
#include <flag_msgs/mode_flag.h>
#include "get_way_points.h"
#include <std_msgs/Int32.h>


geometry_msgs::PoseStamped uav_pose;
sensor_msgs::Imu imu_msg;
geometry_msgs::PoseStamped gt_pose;
airsim_ros::Circle origin_point;
geometry_msgs::PoseStamped temp_pose;
geometry_msgs::PoseStamped windmill;//-39,102,-30
airsim_ros::CirclePoses origin_circle_pt; // 记录静态点的pose
airsim_ros::CirclePoses gt_circle_pt;     // 记录圈坐标的真值
airsim_ros::Circle img_circle_pt;         // 记录感知获得的点
airsim_ros::Circle last_img_circle_pt;    // 缓存感知获得的点
nav_msgs::Path pathcmd;                   // 待发布的路径指令
nav_msgs::Path last_pathcmd;              // 缓存发布的路径指令
std::vector<bool> through_state;
geometry_msgs::PoseStamped back_pt;       // 保存后半段路径点
geometry_msgs::TwistStamped yaw_cmd;
std_msgs::Int32 mode_flag;    // 比赛模式
airsim_ros::Circle circle1;               // 缓存圈1
std_msgs::Int32 target_ID; // 目前正在穿的圈的序号
std_msgs::Int32 last_target_ID; // 目前正在穿的圈的序号
airsim_ros::Circle made_path_circle[2];
nav_msgs::Path way_points;
float yaw = 0;
bool flag = false; //感知更新flag
int through_num = 0;
double pre_z = 0,
       detal_z = 0;
double thrust = 0;
double current_yaw = 0;

double ComputeThrust(double thrust);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "basic_dev"); // 初始化ros 节点，命名为 basic
    ros::NodeHandle n;                  // 创建node控制句柄
    BasicDev go(&n);
    return 0;
}

BasicDev::BasicDev(ros::NodeHandle *nh)
{

    takeoff.request.waitOnLastTask = 1;
    land.request.waitOnLastTask = 1;

    // // 使用publisher发布速度指令需要定义 Velcmd , 并赋予相应的值后，将他publish（）出去
    // velcmd.twist.angular.z = 0; // z方向角速度(yaw, deg)
    // velcmd.twist.linear.x = 0; // x方向线速度(m/s)
    // velcmd.twist.linear.y = 0; // y方向线速度(m/s)
    // velcmd.twist.linear.z = 0; // z方向线速度(m/s)

    // // 使用publisher发布姿态指令需要定义 Posecmd , 并赋予相应的值后，将他publish（）出去
    // posecmd.roll = 0;     // x方向姿态(rad)
    // posecmd.pitch = 0;    // y方向姿态(rad)
    // posecmd.yaw = 0;      // z方向姿态(rad)
    // posecmd.throttle = 0; // 油门， （0.0-1.0）

    // // 使用publisher发布角速度指令需要定义 AngleRateThrottle , 并赋予相应的值后，将他publish（）出去
    // arthrcmd.rollRate = 0;  // roll 角速度（rad/s）
    // arthrcmd.pitchRate = 0; // pitch 角速度 （rad/s）for (int i = 0; i < pathcmd.poses.size(); i++)
    // {
    //     if (i != pathcmd.poses.size() - 1)
    //     {
    //         pathcmd.poses[i] = pathcmd.poses[i + 1];
    //     }
    //     else
    //     {
    //         pathcmd.poses.pop_back();
    //     }
    // }
    // arthrcmd.yawRate = 0;   // yaw 角速度 （rad/s）
    // arthrcmd.throttle = 0;  // 油门， （0.0-1.0）

    // 无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    circles_suber = nh->subscribe<airsim_ros::CirclePoses>("airsim_node/drone_1/circle_poses", 1, std::bind(&BasicDev::origin_circles_cb, this, std::placeholders::_1)); // 障碍圈数据
    odom_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/debug/pose_gt", 1, std::bind(&BasicDev::gt_pose_cb, this, std::placeholders::_1));
    imu_suber = nh->subscribe<sensor_msgs::Imu>("airsim_node/drone_1/imu/imu", 1, std::bind(&BasicDev::imu_cb, this, std::placeholders::_1)); // imu数据

    img_circle_suber = nh->subscribe<airsim_ros::Circle>("/img_circle", 1, std::bind(&BasicDev::img_circle_cb, this, std::placeholders::_1));
    gt_circle_suber = nh->subscribe<airsim_ros::CirclePoses>("/airsim_node/drone_1/debug/circle_poses_gt", 1, std::bind(&BasicDev::gt_circle_cb, this, std::placeholders::_1));

    // 通过这两个服务可以调用模拟器中的无人机起飞和降落命令
    takeoff_client = nh->serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    land_client = nh->serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/land");
    reset_client = nh->serviceClient<airsim_ros::Reset>("/airsim_node/reset");
    // 通过publisher实现对无人机的速度控制和姿态控制和角速度控制
    vel_publisher = nh->advertise<airsim_ros::VelCmd>("airsim_node/drone_1/vel_cmd_body_frame", 1);
    pose_publisher = nh->advertise<airsim_ros::PoseCmd>("airsim_node/drone_1/pose_cmd_body_frame", 1);
    anglerate_publisher = nh->advertise<airsim_ros::AngleRateThrottle>("airsim_node/drone_1/angle_rate_throttle_frame", 1);

    mode_pub = nh->advertise<std_msgs::Int32>("mode_flag", 1);
    path_pub = nh->advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 1);
    count_pub = nh->advertise<std_msgs::Int32>("target_ID", 1);
    yaw_pub = nh->advertise<geometry_msgs::TwistStamped>("/yaw/ctrl", 1);

    //takeoff_client.call(takeoff); // 起飞
    //ros::Duration(2.5).sleep();
    mode_flag.data = 1;
    last_target_ID.data = -1;
    target_ID.data = 0;

    origin_point.position.x = 0;
    origin_point.position.y = 0;
    origin_point.position.z = -1.5;
    origin_point.yaw = 0;

    back_pt.pose.position.x = 0;
    back_pt.pose.position.y = 0;
    back_pt.pose.position.z = -1.5;

    yaw_cmd.twist.linear.z = 0; //yaw

    windmill.pose.position.x = -39;
    windmill.pose.position.y = 102;
    windmill.pose.position.z = -30;
    //Only for debug
    last_check_index  = -1;
    //      land_client.call(land); //降落
    //      reset_client.call(reset); //重置

    // posecmd.throttle = 0.596 + 0.031;
    // posecmd.roll = 0;  // x方向姿态(rad)
    // posecmd.pitch = 0; // y方向姿态(rad
    ros::spinOnce();
    while (ros::ok())
    {
        ros::spinOnce();
        // made_path_circle[0] = origin_circle_pt.poses[target_ID.data];
        // made_path_circle[1] = origin_circle_pt.poses[target_ID.data + 1];
        if(origin_circle_pt.poses.size() < 15)
        {
            ROS_INFO("WAITING CIRCLE MESSAGE>>>>>>>>>>");
        }
        else
        {
            nav_msgs::Path path;
            bool path_change_state = false;
            // made_path_circle[0] = origin_circle_pt.poses[target_ID.data];
            // made_path_circle[1] = origin_circle_pt.poses[target_ID.data + 1];
            if(last_target_ID.data == -1)//segmentation fault
            {
                path_change_state = true;
                target_ID.data = 0;
                last_target_ID.data = target_ID.data;
                made_path_circle[0] = origin_circle_pt.poses[target_ID.data];
                yaw = origin_circle_pt.poses[target_ID.data].yaw;
                last_img_circle_pt = made_path_circle[0];
                //made_path_circle[1] = origin_circle_pt.poses[target_ID.data + 1];
                // ros::spinOnce();
                nav_msgs::Path circle_path = get_1to3_path(made_path_circle[0]);
                geometry_msgs::PoseStamped front_point = circle_path.poses[0];
                Eigen::Vector3d ori_point(origin_point.position.x,origin_point.position.y,origin_point.position.z);
                Eigen::Vector3d frt_point(front_point.pose.position.x,front_point.pose.position.y,front_point.pose.position.z);
                Eigen::Vector3d mid_point = 0.5*(ori_point+frt_point);
                geometry_msgs::PoseStamped tempPoint;
                tempPoint.pose.position.x = mid_point(0);
                tempPoint.pose.position.y = mid_point(1);
                tempPoint.pose.position.z = mid_point(2);
                path.poses.push_back(tempPoint);
                for(int i = 0;i<circle_path.poses.size();i++)
                {
                    path.poses.push_back(circle_path.poses[i]);
                }
                through_state.clear();
                through_state.resize(path.poses.size(),false);
            }
            else
            {   
                bool circle_change = JudgeCircleThroughState();
                if(last_target_ID.data != target_ID.data && circle_change)
                {
                    ROS_INFO("PASS CIRCLE!!");
                    ROS_INFO("TARGET DATA NOW:%d",target_ID.data);
                    //last_img_circle_pt = made_path_circle[0];
                    last_img_circle_pt = made_path_circle[0];
                    path_change_state = true;
                    back_pt = last_pathcmd.poses[last_pathcmd.poses.size()-1];
                    made_path_circle[0] = origin_circle_pt.poses[target_ID.data];
                    yaw = origin_circle_pt.poses[target_ID.data].yaw;
                    //made_path_circle[1] = origin_circle_pt.poses[target_ID.data + 1];
                    path = get_path(last_img_circle_pt, made_path_circle[0]);
                    last_target_ID.data = target_ID.data;
                    through_state.clear();
                    through_state.resize(path.poses.size(),false);
                }
                else if(!circle_change)
                {
                    if(target_ID.data == 0)
                    {
                        //made_path_circle[0] = origin_circle_pt.poses[target_ID.data];
                       
                        nav_msgs::Path circle_path = get_1to3_path(made_path_circle[0]);
                        geometry_msgs::PoseStamped front_point = circle_path.poses[0];
                        Eigen::Vector3d ori_point(origin_point.position.x,origin_point.position.y,origin_point.position.z);
                        Eigen::Vector3d frt_point(front_point.pose.position.x,front_point.pose.position.y,front_point.pose.position.z);
                        Eigen::Vector3d mid_point = 0.5*(ori_point+frt_point);
                        geometry_msgs::PoseStamped tempPoint;
                        tempPoint.pose.position.x = mid_point(0);
                        tempPoint.pose.position.y = mid_point(1);
                        tempPoint.pose.position.z = mid_point(2);
                        path.poses.push_back(tempPoint);
                        for(int i = 0;i<circle_path.poses.size();i++)
                        {
                            path.poses.push_back(circle_path.poses[i]);
                        }
                        path_change_state = JudgePointThroughState();
                    }
                    else
                    {
                        path = get_path(last_img_circle_pt, made_path_circle[0]);
                        path_change_state = JudgePointThroughState();
                    }
                   
                }
            }
            nav_msgs::Path path_cmd;
            for(int i = 0;i<through_state.size();i++)
            {
                if(!through_state[i])
                {
                    path_cmd.poses.push_back(path.poses[i]);
                }
            }
            if (path_change_state)
            {
                //ROS_INFO("PASSING CHANGE STATE!!");
                path_cmd.header.frame_id = "odom";
                path_cmd.header.stamp = ros::Time::now();
                if (path_cmd.poses.size() > 1)
                {
                    path_pub.publish(path_cmd);
                }
                ROS_INFO("POINTS NUMBER:%ld",path_cmd.poses.size());
                last_pathcmd = path_cmd;
            }
            
            YawPublish();
            count_pub.publish(target_ID);
        }
        ros::spinOnce();
    }
}
// std::cout << "Current yaw : " << current_yaw * 180.0 / M_PI << std::endl;

// ROS_INFO("1");
// ros::Duration(0.12).sleep();

// std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

// posecmd.yaw = 0;
// while (ros::ok() && i < 12)
// {
//     pose_publisher.publish(posecmd);
//     ros::spinOnce();
//     // ROS_INFO("2");
//     ros::Duration(0.12).sleep();
//     i++;
// }
// i = 0;

// posecmd.yaw = -PI / 6;
// while (ros::ok() && i < 12)
// {
//     pose_publisher.publish(posecmd);
//     ros::spinOnce();
//     // ROS_INFO("3");
//     ros::Duration(0.12).sleep();
//     i++;
// }
// i = 0;

// posecmd.yaw = 0;
// while (ros::ok() && i < 12)
// {    // ROS_INFO("Get circle poses data.");
//     pose_publisher.publish(posecmd);
//     ros::spinOnce();
//     // ROS_INFO("4");
//     ros::Duration(0.12).sleep();
//     i++;
// }
// i = 0;

// ros::Rate loop_rate(20);
// while (ros::ok())
// {
//     ros::spinOnce();
// }

BasicDev::~BasicDev()
{
}

void BasicDev::imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu_msg = *msg;
}

void BasicDev::origin_circles_cb(const airsim_ros::CirclePoses::ConstPtr &msg)
{
    origin_circle_pt = *msg;
}

void BasicDev::gt_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    gt_pose = *msg;
}

void BasicDev::gt_circle_cb(const airsim_ros::CirclePoses::ConstPtr &msg)
{
    gt_circle_pt = *msg;
}

void BasicDev::img_circle_cb(const airsim_ros::Circle::ConstPtr &msg)
{
    last_img_circle_pt = img_circle_pt;
    img_circle_pt = *msg;
    if (img_circle_pt.yaw == 0)
    {
        img_circle_pt.yaw = origin_circle_pt.poses[target_ID.data].yaw;
    }
    // 检查感知圈位置和之前发送的圈位置 是否一致
    if ((abs(last_img_circle_pt.position.x - img_circle_pt.position.x) > 0.1) || 
    (abs(last_img_circle_pt.position.y - img_circle_pt.position.y) > 0.1) || 
    (abs(last_img_circle_pt.position.z - img_circle_pt.position.z) > 0.1) || 
    (abs(last_img_circle_pt.yaw - img_circle_pt.yaw) > 5))
    {
        Eigen::Vector3d post_p(origin_circle_pt.poses[target_ID.data].position.x,origin_circle_pt.poses[target_ID.data].position.y,origin_circle_pt.poses[target_ID.data].position.z);
        Eigen::Vector3d des_p(img_circle_pt.position.x,img_circle_pt.position.y,img_circle_pt.position.z);
        Eigen::Vector3d dis_p = des_p-post_p;
        if (dis_p.norm() < 4)
        {
            flag = true;
            made_path_circle[0] = img_circle_pt;
            yaw = img_circle_pt.yaw;
        }   
        std::cout << made_path_circle[0] <<std::endl;
    }
}

nav_msgs::Path BasicDev::get_1to3_path(const airsim_ros::Circle circle_position)
{
    nav_msgs::Path path_3;
    geometry_msgs::PoseStamped position_1, position_2, position_3, circle_pose, position_4, position_5, position_6;

    float circle_yaw = -circle_position.yaw;
    float distant_1 = 5, distant_2 = 6.5, distant_3 = 6;
    if (target_ID.data == 4)
    {
        distant_1 = 5;
        distant_2 = 20;
    }
    // circle_yaw -= M_PI / 2;
    // if (circle_yaw > M_PI)
    //     circle_yaw -= 2 * M_PI;
    // else if (circle_yaw < -M_PI)
    // {
    //     circle_yaw += 2 * M_PI;
    // }
    circle_yaw = circle_yaw / 180.0 * M_PI;

    position_1.pose.position.x = circle_position.position.x - distant_3 * cos(circle_yaw);
    position_1.pose.position.y = circle_position.position.y + distant_3 * sin(circle_yaw);
    position_1.pose.position.z = circle_position.position.z;

    position_2.pose.position.x = circle_position.position.x - distant_2 * cos(circle_yaw);
    position_2.pose.position.y = circle_position.position.y + distant_2 * sin(circle_yaw);
    position_2.pose.position.z = circle_position.position.z;

    position_3.pose.position.x = circle_position.position.x - distant_1 * cos(circle_yaw);
    position_3.pose.position.y = circle_position.position.y + distant_1 * sin(circle_yaw);
    position_3.pose.position.z = circle_position.position.z;

    position_4.pose.position.x = circle_position.position.x + distant_1 * cos(circle_yaw);
    position_4.pose.position.y = circle_position.position.y - distant_1 * sin(circle_yaw);
    position_4.pose.position.z = circle_position.position.z;

    position_5.pose.position.x = circle_position.position.x + distant_2 * cos(circle_yaw);
    position_5.pose.position.y = circle_position.position.y - distant_2 * sin(circle_yaw);
    position_5.pose.position.z = circle_position.position.z;

    position_6.pose.position.x = circle_position.position.x + distant_3 * cos(circle_yaw);
    position_6.pose.position.y = circle_position.position.y - distant_3 * sin(circle_yaw);
    position_6.pose.position.z = circle_position.position.z;

    circle_pose.pose.position.x = circle_position.position.x;
    circle_pose.pose.position.y = circle_position.position.y;
    circle_pose.pose.position.z = circle_position.position.z;

    if (target_ID.data == 16)
    {
         // path_3.poses.push_back(position_1);
        // path_3.poses.push_back(position_2);
        path_3.poses.push_back(position_3);
        path_3.poses.push_back(circle_pose);
        path_3.poses.push_back(position_4);
        // path_3.poses.push_back(position_5);
        //  path_3.poses.push_back(position_6);
    }
    else if (target_ID.data == 4)
    {
        if ((pow((position_1.pose.position.x - origin_circle_pt.poses[target_ID.data + 1].position.x), 2) + pow((position_1.pose.position.y - origin_circle_pt.poses[target_ID.data + 1].position.y), 2) + pow((position_1.pose.position.z - origin_circle_pt.poses[target_ID.data + 1].position.z), 2)) > (pow((position_6.pose.position.x - origin_circle_pt.poses[target_ID.data + 1].position.x), 2) + pow((position_6.pose.position.y - origin_circle_pt.poses[target_ID.data + 1].position.y), 2) + pow((position_6.pose.position.z - origin_circle_pt.poses[target_ID.data + 1].position.z), 2)))
        {
            // path_3.poses.push_back(position_1);
            path_3.poses.push_back(position_2);
            path_3.poses.push_back(position_3);
            path_3.poses.push_back(circle_pose);
            path_3.poses.push_back(position_4);
            //path_3.poses.push_back(position_5);
            //  path_3.poses.push_back(position_6);
        }
        else
        {
            // path_3.poses.push_back(position_6);
            path_3.poses.push_back(position_5);
            path_3.poses.push_back(position_4);
            path_3.poses.push_back(circle_pose);
            path_3.poses.push_back(position_3);
            //path_3.poses.push_back(position_2);
            //  path_3.poses.push_back(position_1);
        }
    }
    else if (target_ID.data == 3)
    {
        path_3.poses.push_back(position_3);
        path_3.poses.push_back(circle_pose);
        path_3.poses.push_back(position_4);
    }
    else if ((pow((position_1.pose.position.x - origin_circle_pt.poses[target_ID.data + 1].position.x), 2) + pow((position_1.pose.position.y - origin_circle_pt.poses[target_ID.data + 1].position.y), 2) + pow((position_1.pose.position.z - origin_circle_pt.poses[target_ID.data + 1].position.z), 2)) > (pow((position_6.pose.position.x - origin_circle_pt.poses[target_ID.data + 1].position.x), 2) + pow((position_6.pose.position.y - origin_circle_pt.poses[target_ID.data + 1].position.y), 2) + pow((position_6.pose.position.z - origin_circle_pt.poses[target_ID.data + 1].position.z), 2)))
    {
        // path_3.poses.push_back(position_1);
        // path_3.poses.push_back(position_2);
        path_3.poses.push_back(position_3);
        path_3.poses.push_back(circle_pose);
        path_3.poses.push_back(position_4);
        // path_3.poses.push_back(position_5);
        //  path_3.poses.push_back(position_6);
    }
    else
    {
        // path_3.poses.push_back(position_6);
        // path_3.poses.push_back(position_5);
        path_3.poses.push_back(position_4);
        path_3.poses.push_back(circle_pose);
        path_3.poses.push_back(position_3);
        // path_3.poses.push_back(position_2);
        //  path_3.poses.push_back(position_1);
    }
    return path_3;
}

geometry_msgs::PoseStamped BasicDev::get_midpoint(const nav_msgs::Path fron_path, const nav_msgs::Path behi_path)
{
    geometry_msgs::PoseStamped midpoint;
    midpoint.pose.position.x = (4 * fron_path.poses[fron_path.poses.size() - 1].pose.position.x + 3 * behi_path.poses[0].pose.position.x) / 7;
    midpoint.pose.position.y = (4 * fron_path.poses[fron_path.poses.size() - 1].pose.position.y + 3 * behi_path.poses[0].pose.position.y) / 7;
    midpoint.pose.position.z = (4 * fron_path.poses[fron_path.poses.size() - 1].pose.position.z + 3 * behi_path.poses[0].pose.position.z) / 7;
    // midpoint.pose.position.x = fron_path.poses[fron_path.poses.size()].pose.position.x;
    // midpoint.pose.position.y = fron_path.poses[fron_path.poses.size()].pose.position.y;
    // midpoint.pose.position.z = fron_path.poses[fron_path.poses.size()].pose.position.z;

    return midpoint;
}

nav_msgs::Path BasicDev::get_path(airsim_ros::Circle made_path_circle_0, airsim_ros::Circle made_path_circle_1)
{
    airsim_ros::Circle fron_circle = made_path_circle_0;
    airsim_ros::Circle behi_circle = made_path_circle_1;
    nav_msgs::Path fron_1to3_path, behi_1to3_path;

    fron_1to3_path = get_1to3_path(fron_circle);
    behi_1to3_path = get_1to3_path(behi_circle);
    
    nav_msgs::Path path;
    path.poses.push_back(back_pt);

    if (target_ID.data == 2)
    {
        path.poses.erase(path.poses.end ());
    }

    geometry_msgs::PoseStamped midpoint;
    if (target_ID.data == 15)
    {
        midpoint = windmill;
    }
    else
    {
        midpoint = get_midpoint(fron_1to3_path, behi_1to3_path);
    }
    path.poses.push_back(midpoint);
    if (target_ID.data == 3)
    {
        path.poses.erase(path.poses.end ());
    }

    if (target_ID.data == 12)
    {
        geometry_msgs::PoseStamped midpoint_2;
        airsim_ros::Circle _midpoint;
        _midpoint.position.x = midpoint.pose.position.x;
        _midpoint.position.y = midpoint.pose.position.y;
        _midpoint.position.z = midpoint.pose.position.z;
        midpoint_2 = get_midpoint(get_1to3_path(_midpoint), behi_1to3_path);
        path.poses.push_back(midpoint_2);
    }
    // if (target_ID.data == 3 || target_ID.data == 13)
    // {
    //     path.poses.erase(path.poses.end()); //去除midpoint
    // }
    // cv::Point3d p0 = {fron_1to3_path.poses[fron_1to3_path.poses.size() - 1].pose.position.x, 
    //                     fron_1to3_path.poses[fron_1to3_path.poses.size() - 1].pose.position.y, 
    //                     fron_1to3_path.poses[fron_1to3_path.poses.size() - 1].pose.position.z}; // 前圈后点
    // cv::Point3d p1 = {behi_1to3_path.poses[0].pose.position.x, 
    //                     behi_1to3_path.poses[0].pose.position.y, 
    //                     behi_1to3_path.poses[0].pose.position.z};                                                                                           // 后圈前点
    // cv::Point3d p2 = {behi_1to3_path.poses[behi_1to3_path.poses.size() - 1].pose.position.x, 
    //                     behi_1to3_path.poses[behi_1to3_path.poses.size() - 1].pose.position.y, 
    //                     behi_1to3_path.poses[behi_1to3_path.poses.size() - 1].pose.position.z}; // 后圈后点

    // double yaw = gt_circle_pt.poses[target_ID.data].yaw / 180 * M_PI;
    // std::vector<cv::Point3d> waypoint_vector;
    // robot_decision::get_way_points(p0, p2, p1, yaw, waypoint_vector); // yaw角为弧度制,为无人机当前朝向，目前用圈的朝向代替
    //     for (int i = 0; i < waypoint_vector.size(); i++)
    //     {
    //         geometry_msgs::PoseStamped way_pt;
    //         way_pt.pose.position.x = waypoint_vector[i].x;
    //         way_pt.pose.position.y = waypoint_vector[i].y;
    //         way_pt.pose.position.z = waypoint_vector[i].z;
    //         path.poses.push_back(way_pt);
    //     }
        for (int i = 0; i < behi_1to3_path.poses.size(); i++)
        {
            path.poses.push_back(behi_1to3_path.poses[i]);
        }
    
    return path;
}
bool BasicDev::JudgeCircleThroughState()
{
    Eigen::Vector3d post_p(gt_pose.pose.position.x,gt_pose.pose.position.y,gt_pose.pose.position.z);
    Eigen::Vector3d des_p(img_circle_pt.position.x,img_circle_pt.position.y,img_circle_pt.position.z);
    Eigen::Vector3d dis_p = des_p-post_p;
    // ROS_INFO("TARGET ID CIRCLE!%d: %f",target_ID.data,dis_p.norm());
    if(dis_p.norm()<0.8 && flag)
    {
        switch(target_ID.data)
        {
            case 4:
            {
                target_ID.data = 5;
                mode_flag.data = 2;
                break;
            }
            case 5:
            {
                target_ID.data = 6;
                break;
            }
            case 6:
            {
                target_ID.data = 7;
                break;
            }
            case 7:
            {
                target_ID.data = 12;
                mode_flag.data = 3;
                break;
            }
            default:
            {
                target_ID.data++;
                break;
            }
        }
        flag = false;

        ROS_INFO("TARGET ID CHANGE!%d",target_ID.data);
        return true;
        //back_pt = pathcmd.poses[pathcmd.poses.size() - 1];
    }
    else
    {
        return false;
    }

}
bool BasicDev::JudgePointThroughState()
{
    int check_point_index = 0;
    for(int i = 0;i<through_state.size();i++)
    {
        if(!through_state[i]) 
        {
            check_point_index = i;
            break;
        }
    }
    if(last_check_index!=check_point_index)
    {
        ROS_INFO("CHECK POINT INDEX:%d",check_point_index);
        last_check_index = check_point_index;
    }   
    Eigen::Vector3d post_p(gt_pose.pose.position.x,gt_pose.pose.position.y,gt_pose.pose.position.z);
    Eigen::Vector3d des_p(last_pathcmd.poses[0].pose.position.x,last_pathcmd.poses[0].pose.position.y,last_pathcmd.poses[0].pose.position.z);
    Eigen::Vector3d dis_p = des_p-post_p;
    // if(check_point_index ==2)
    // {
    //     ROS_INFO("TARGET DISTANCE:%f",dis_p.norm());
    // }
    if(dis_p.norm()<0.8)
    {
        through_state[check_point_index]  = true;
        return true;
    }
    else
    {
        return false;
    }
}

void BasicDev::YawPublish()
{
    float target_yaw = 0;

    target_yaw = atan2((made_path_circle[0].position.y - gt_pose.pose.position.y) , (made_path_circle[0].position.x - gt_pose.pose.position.x));
    // if (target_ID.data < 4 )
    // {
    //     target_yaw = yaw / 180.0 * M_PI;
    // }
    // else
    // {
    //     target_yaw = (yaw + 180.0) / 180.0 * M_PI;
    // }
    
    // double current_yaw = tf::getYaw(gt_pose.pose.orientation); //debug

    // // ROS_INFO("TARGET YAW:%f",target_yaw);
    // // ROS_INFO("CURRENT YAW:%f",current_yaw);
    // if (abs(target_yaw - current_yaw) > 0.1)
    // {
    //     if (target_yaw - current_yaw > 0)
    //     {
    //         yaw_cmd.twist.linear.z = current_yaw + 0.05;
    //     }
    //     if (target_yaw - current_yaw < 0)
    //     {
    //         yaw_cmd.twist.linear.z = current_yaw - 0.05;
    //     }
    // }
    // else 
    // {
         yaw_cmd.twist.linear.z = target_yaw;
    // }
    yaw_pub.publish(yaw_cmd);
}

#endif
