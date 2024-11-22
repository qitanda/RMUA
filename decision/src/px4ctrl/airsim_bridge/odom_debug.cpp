#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <Eigen/Eigen>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <queue>
ros::Publisher gt_odom_pub;
Eigen::Vector3d gt_p,gt_v;
Eigen::Quaterniond gt_q;
double stamp_last,stamp_now;
std::deque<Eigen::Vector3d> pose_quene;
std::deque<double> time_quene;
void SimulatorPoseCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    //stamp_last = stamp_now;
    stamp_now = ros::Time::now().toSec();
    gt_p(0) = msg->pose.position.x;
    gt_p(1) = msg->pose.position.y;
    gt_p(2) = msg->pose.position.z;
    gt_q = Eigen::Quaterniond(Eigen::Vector4d(msg->pose.orientation.x,msg->pose.orientation.y,
                                              msg->pose.orientation.z,msg->pose.orientation.w));
    Eigen::Vector3d pose_tmp = gt_p;
    if(pose_quene.size()<10 && time_quene.size()<10)
    {
        std::cout<<"QUEUE SIZE"<<pose_quene.size()<<std::endl;
        pose_quene.emplace_back(pose_tmp);
        time_quene.emplace_back(stamp_now);
        gt_v(0)=0;
        gt_v(1)=0;
        gt_v(2)=0;
    }
    else
    {
        pose_quene.pop_front();
        time_quene.pop_front();
        pose_quene.emplace_back(pose_tmp);
        time_quene.emplace_back(stamp_now);
        Eigen::Vector3d start_p = pose_quene[pose_quene.size()-2];
        Eigen::Vector3d end_p = pose_quene[pose_quene.size()-1];
        Eigen::Vector3d dis_p = end_p-start_p;
        std::cout<<"start_p P "<<start_p.transpose()<<std::endl;
        std::cout<<"end_p P "<<end_p.transpose()<<std::endl;
        std::cout<<"DIS P "<<dis_p.transpose()<<std::endl;
        std::cout<<"TIME STEP"<<time_quene[time_quene.size()-1]-time_quene[time_quene.size()-2]<<std::endl;
        gt_v = dis_p/(time_quene[time_quene.size()-1]-time_quene[time_quene.size()-2]);
        for(int i =0;i<3;i++)
        {  
            if(abs(gt_v(i)) < 0.02) gt_v(i) = 0;
        }
    }  
    nav_msgs::Odometry gt_odom;
    gt_odom.header.frame_id = "map";
    gt_odom.header.stamp = ros::Time::now();
    gt_odom.pose.pose.position.x = gt_p(0);
    gt_odom.pose.pose.position.y = gt_p(1);
    gt_odom.pose.pose.position.z = gt_p(2);
    gt_odom.pose.pose.orientation.x = gt_q.x();
    gt_odom.pose.pose.orientation.y = gt_q.y();
    gt_odom.pose.pose.orientation.z = gt_q.z();
    gt_odom.pose.pose.orientation.w = gt_q.w();
    gt_odom.twist.twist.linear.x = gt_v.x();
    gt_odom.twist.twist.linear.y = gt_v.y();
    gt_odom.twist.twist.linear.z = gt_v.z();
    double roll,pitch,yaw;
    Eigen::Quaterniond output_q = gt_q;
    tf2::Quaternion tf_q(output_q.x(),output_q.y(),output_q.z(),output_q.w());
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);// 四元数转欧拉角
    //ROS_INFO("roll=%.2f, pitch=%.2f, yaw=%.2f", roll, pitch, yaw);
    gt_odom_pub.publish(gt_odom);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_debug");
    ros::NodeHandle nh;
    stamp_now = ros::Time::now().toSec();
    ros::Subscriber gt_odom_sub = nh.subscribe("/airsim_node/drone_1/debug/pose_gt",1,SimulatorPoseCallback);
    gt_odom_pub = nh.advertise<nav_msgs::Odometry>("/airsim_bridge/drone_1/debug/odom", 1);
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
        return 0;
}