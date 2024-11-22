#ifndef PATH_GENERATOR_H
#define PATH_GENERATOR_H


#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <deque>
#include <boost/format.hpp>
#include <eigen3/Eigen/Dense>
using namespace std;
class WayPointGenerator
{
private:
    ros::Publisher path_pub,path_vis_pub;
    ros::Subscriber odom_sub;
    nav_msgs::Path waypoints_;
    geometry_msgs::PoseStamped begin_point_;
    ros::NodeHandle nh_;
    int waypoint_num;
    int path_type_;
public:
    WayPointGenerator(ros::NodeHandle &nh);
    ~WayPointGenerator(){};
    void PublishWayPoints();
    void GenerateWaypoints();
    void OutputAllWayPoints();
};
#endif