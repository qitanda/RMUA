#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

double height = 1.0;
double radius = 1.0;
double scale = 1.0;
double eight_scale = 0.7;
double x_offset = 0.0, y_offset = 0, z_offset = 0;

void AddWaypoint(nav_msgs::Path &path, double x, double y, double z)
{
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    pt.pose.position.y = x_offset + scale * x;
    pt.pose.position.x = y_offset + scale * y;
    pt.pose.position.z = height + z;
    path.poses.push_back(pt);
}

// circle traj
nav_msgs::Path CircleTraj(void)
{
    nav_msgs::Path waypoints;
    AddWaypoint(waypoints, 0, 0, 0);
    AddWaypoint(waypoints, radius-0.707*radius, 0.707*radius, 0);
    AddWaypoint(waypoints, radius, radius, 0);
    AddWaypoint(waypoints, radius+0.707*radius, 0.707*radius, 0);
    AddWaypoint(waypoints, 2*radius, 0, 0);
    AddWaypoint(waypoints, radius+0.707*radius, -0.707*radius, 0);
    AddWaypoint(waypoints, radius, -radius, 0);
    AddWaypoint(waypoints, radius-0.707*radius, -0.707*radius, 0);
    AddWaypoint(waypoints, 0, 0, 0);
    return waypoints;
}

// eight traj
nav_msgs::Path EightTraj(void)
{
    nav_msgs::Path waypoints;
    AddWaypoint(waypoints, 0, 0, 0);
    AddWaypoint(waypoints, radius-0.707*radius, 0.707*radius*eight_scale, 0);
    AddWaypoint(waypoints, radius, radius*eight_scale, 0);
    AddWaypoint(waypoints, radius+0.707*radius, 0.707*radius*eight_scale, 0);
    AddWaypoint(waypoints, 2*radius, 0, 0);
    AddWaypoint(waypoints, radius+0.707*radius, -0.707*radius*eight_scale, 0);
    AddWaypoint(waypoints, radius, -radius*eight_scale, 0);
    AddWaypoint(waypoints, radius-0.707*radius, -0.707*radius*eight_scale, 0);
    AddWaypoint(waypoints, 0, 0, 0);
    AddWaypoint(waypoints, -radius+0.707*radius, 0.707*radius*eight_scale, 0);
    AddWaypoint(waypoints, -radius, radius*eight_scale, 0);
    AddWaypoint(waypoints, -radius-0.707*radius, 0.707*radius*eight_scale, 0);
    AddWaypoint(waypoints, -2*radius, 0, 0);
    AddWaypoint(waypoints, -radius-0.707*radius, -0.707*radius*eight_scale, 0);
    AddWaypoint(waypoints, -radius, -radius*eight_scale, 0);
    AddWaypoint(waypoints, -radius+0.707*radius, -0.707*radius*eight_scale, 0);

    AddWaypoint(waypoints, 0, 0, 0);
    return waypoints;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "special_traj_node");
    ros::NodeHandle nh;

    nh.param("/special_traj_node/height", height, 1.0);
    nh.param("/special_traj_node/radius", radius, 1.0);
    nh.param("/special_traj_node/scale", scale, 1.0);
    nh.param("/special_traj_node/eight_scale", eight_scale, 1.0);

    ros::Publisher pub = nh.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 1);
    ros::Duration(2.0).sleep();

    nav_msgs::Path msg = EightTraj();
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    pub.publish(msg);

    ros::spin();
    return 0;
}