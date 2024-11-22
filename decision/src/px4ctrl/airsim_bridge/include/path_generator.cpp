#include "path_generator.h"
using namespace std;

WayPointGenerator::WayPointGenerator(ros::NodeHandle &nh) : nh_(nh)
{
    path_pub = nh_.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 50);
    path_vis_pub = nh_.advertise<geometry_msgs::PoseArray>("waypoints_vis", 10);
    nh_.param("waypoint_num", waypoint_num,6);
    nh_.param("path_num", path_type_,1);
    std::cout<<"WATPOINT NUM: "<<waypoint_num<<std::endl;
    std::cout<<"PATH TYPE: "<<path_type_<<std::endl;

}
void WayPointGenerator::GenerateWaypoints()
{
    waypoints_.poses.clear();
    geometry_msgs::PoseStamped point;
    double circle_R = 1.0;
    begin_point_.pose.orientation.w=1.0;
    begin_point_.pose.orientation.x=0.0;
    begin_point_.pose.orientation.y=0.0;
    begin_point_.pose.orientation.z=0.0;
    switch(path_type_)
    {
        case 0:
            //Test Generate Several Waypoints
            begin_point_.pose.position.x = 0.0;
            begin_point_.pose.position.y = 0.0;
            begin_point_.pose.position.z = -1.5;
            waypoints_.poses.push_back(begin_point_);
            begin_point_.pose.position.x = 0.0;
            begin_point_.pose.position.y = 2.0;
            begin_point_.pose.position.z = -1.5;
            waypoints_.poses.push_back(begin_point_);
            begin_point_.pose.position.x = 2.0;
            begin_point_.pose.position.y = 2.0;
            begin_point_.pose.position.z = -1.5;
            waypoints_.poses.push_back(begin_point_);
            begin_point_.pose.position.x = 2.0;
            begin_point_.pose.position.y = 4.0;
            begin_point_.pose.position.z = -1.5;
            waypoints_.poses.push_back(begin_point_);
            begin_point_.pose.position.x = 2.0;
            begin_point_.pose.position.y = 4.0;
            begin_point_.pose.position.z = -3.0;
            waypoints_.poses.push_back(begin_point_);
            begin_point_.pose.position.x = 4.0;
            begin_point_.pose.position.y = 4.0;
            begin_point_.pose.position.z = -1.5;
            waypoints_.poses.push_back(begin_point_);
        break;
        case 1:
            begin_point_.pose.position.x = 0.0;
            begin_point_.pose.position.y = 0.0;
            begin_point_.pose.position.z = -1.5;
            waypoints_.poses.push_back(begin_point_);
            // begin_point_.pose.position.x = 0.0;
            // begin_point_.pose.position.y = 2.0;
            // begin_point_.pose.position.z = 0.8;
            // waypoints_.poses.push_back(begin_point_);
            begin_point_.pose.position.x = 0.0;
            begin_point_.pose.position.y = 3.0;
            begin_point_.pose.position.z = -1.5;
            waypoints_.poses.push_back(begin_point_);
            // begin_point_.pose.position.x = 0.0;
            // begin_point_.pose.position.y = 4.0;
            // begin_point_.pose.position.z = 0.8;
            // waypoints_.poses.push_back(begin_point_);
            begin_point_.pose.position.x = 2.0;
            begin_point_.pose.position.y = 5.0;
            begin_point_.pose.position.z = -1.5;
            waypoints_.poses.push_back(begin_point_);
            // begin_point_.pose.position.x = 0.0;
            // begin_point_.pose.position.y = 6.0;
            // begin_point_.pose.position.z = 0.8;
            // waypoints_.poses.push_back(begin_point_);
            begin_point_.pose.position.x = 1.0;
            begin_point_.pose.position.y = 7.0;
            begin_point_.pose.position.z = -1.5;
            waypoints_.poses.push_back(begin_point_);
            // begin_point_.pose.position.x = 0.0;
            // begin_point_.pose.position.y = 8.0;
            // begin_point_.pose.position.z = 0.8;
            // waypoints_.poses.push_back(begin_point_);
            begin_point_.pose.position.x = 0.0;
            begin_point_.pose.position.y = 9.0;
            begin_point_.pose.position.z = -1.5;
            waypoints_.poses.push_back(begin_point_);
        break;
        case 2:
            begin_point_.pose.position.x = 0.0;
            begin_point_.pose.position.y = 0.0;
            begin_point_.pose.position.z = -1.5;
            waypoints_.poses.push_back(begin_point_);
            // begin_point_.pose.position.x = 0.0;
            // begin_point_.pose.position.y = 2.0;
            // begin_point_.pose.position.z = 0.8;
            // waypoints_.poses.push_back(begin_point_);
            begin_point_.pose.position.x = 3.0;
            begin_point_.pose.position.y = 2.0;
            begin_point_.pose.position.z = -1.5;
            waypoints_.poses.push_back(begin_point_);
            // begin_point_.pose.position.x = 0.0;
            // begin_point_.pose.position.y = 4.0;
            // begin_point_.pose.position.z = 0.8;
            // waypoints_.poses.push_back(begin_point_);
            begin_point_.pose.position.x = 5.0;
            begin_point_.pose.position.y = 2.0;
            begin_point_.pose.position.z = -1.5;
            waypoints_.poses.push_back(begin_point_);
            // begin_point_.pose.position.x = 0.0;
            // begin_point_.pose.position.y = 6.0;
            // begin_point_.pose.position.z = 0.8;
            // waypoints_.poses.push_back(begin_point_);
            begin_point_.pose.position.x = 7.0;
            begin_point_.pose.position.y = 0.0;
            begin_point_.pose.position.z = -1.5;
            waypoints_.poses.push_back(begin_point_);
            // begin_point_.pose.position.x = 0.0;
            // begin_point_.pose.position.y = 8.0;
            // begin_point_.pose.position.z = 0.8;
            // waypoints_.poses.push_back(begin_point_);
            begin_point_.pose.position.x = 9.0;
            begin_point_.pose.position.y = -2.0;
            begin_point_.pose.position.z = -1.5;
            waypoints_.poses.push_back(begin_point_);
            break;
    }

}
void WayPointGenerator::OutputAllWayPoints()
{
    for(int i = 0;i<waypoints_.poses.size();i++)
    {
        std::cout<<"Point "<< i <<" x: "<<waypoints_.poses[i].pose.position.x
                                <<" y: "<<waypoints_.poses[i].pose.position.y
                                <<" z: "<<waypoints_.poses[i].pose.position.z
                                <<std::endl;
    }
}
void WayPointGenerator::PublishWayPoints()
{
    waypoints_.header.frame_id = std::string("map");
    waypoints_.header.stamp = ros::Time::now();
    path_pub.publish(waypoints_);
    nav_msgs::Path wp_vis = waypoints_;
    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = std::string("map");
    poseArray.header.stamp = ros::Time::now();

    // {
    //     geometry_msgs::Pose init_pose;
    //     init_pose = odom.pose.pose;
    //     poseArray.poses.push_back(init_pose);
    // }

    for (auto it = waypoints_.poses.begin(); it != waypoints_.poses.end(); ++it) {
        geometry_msgs::Pose p;
        p = it->pose;
        poseArray.poses.push_back(p);
    }
    path_vis_pub.publish(poseArray);

}