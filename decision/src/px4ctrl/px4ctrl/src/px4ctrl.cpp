#include "px4ctrl.h"

PX4CrtlNode::PX4CrtlNode(ros::NodeHandle &nh) : nh_(nh)
{
    // Initialization
    param_.Init(nh_);
    bd_.Init(nh_);
    ctrl_.Reset(param_);
    ros::Duration(1).sleep();

    // variable initialize
    hover_des_.p = Eigen::Vector3d(0, 0, 0);
    hover_des_.v = Eigen::Vector3d(0, 0, 0);
    hover_des_.a = Eigen::Vector3d(0, 0, 0);
    hover_des_.j = Eigen::Vector3d(0, 0, 0); 
    hover_des_.yaw = 0;
    hover_des_.yaw_rate = 0;
    
    takeoff.request.waitOnLastTask = 1;
    land.request.waitOnLastTask = 1;
    // PX4 and simulator initialize
    if (bd_.airsim_flag_) { // for airsim simulator
        ROS_INFO("\033[32m[PX4CTRL] Simulator Running! \033[32m");
        bd_.mode_ = Command;
        airsim_pub_ = nh_.advertise<airsim_ros::PoseCmd>("/airsim_node/drone_1/pose_cmd_body_frame", 1);
         //通过这两个服务可以调用模拟器中的无人机起飞和降落命令
        takeoff_client = nh_.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
        land_client = nh_.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/land");
        reset_client = nh_.serviceClient<airsim_ros::Reset>("/airsim_node/reset");
        //通过publisher实现对无人机的速度控制和姿态控制和角速度控制
        takeoff_client.call(takeoff); //起飞
        ros::Duration(2).sleep();
    } 
    ctrl_timer_ = nh_.createTimer(ros::Duration(1.0/param_.freq_), &PX4CrtlNode::CtrlTimerCallback, this, false, true);
}

void PX4CrtlNode::CtrlTimerCallback(const ros::TimerEvent &)
{
    ros::Time t_now = ros::Time::now();

    DesireState_t des_state;

    if ((bd_.odom_.stamp_ - t_now).toSec() > 0.5) return;
    // STEP1: FSM
    if (bd_.airsim_flag_) {
        if (bd_.cmd_.debug_cmd_flag) des_state = GetCmdDes();
        else des_state.p.z() = -1.5;
    } 
    // STEP2: estimate thrust model
    if (bd_.mode_ == Hover || bd_.mode_ == Command) {
        ctrl_.estimateThrustModel(bd_.imu_.a, bd_.battery_.volt, param_);
    }
    
    // STEP3: solve and update new control commands
    ctrl_.RotorDragLoop(des_state, u, bd_, param_);

    // STEP4: publish control commands to airsim
    if (param_.bodyrate_flag_) {
        BodyrateCtrlPub(u, t_now);
    } else {
        AttitudeCtrlPub(u, t_now);
    }
}

DesireState_t PX4CrtlNode::GetCmdDes(void)
{
    DesireState_t des;
    des.p = bd_.cmd_.p;
    des.v = bd_.cmd_.v;
    des.a = bd_.cmd_.a;
    des.j = bd_.cmd_.j; 
    des.yaw = bd_.cmd_.yaw;
    //des.yaw_rate = bd_.cmd_.yaw_rate;
    return des;
}

void PX4CrtlNode::AttitudeCtrlPub(const CtrlOutput_t &u, const ros::Time &stamp)
{
    if(bd_.airsim_flag_) {
        airsim_ros::PoseCmd posecmd;
        double roll,pitch,yaw;
        Eigen::Quaterniond output_q = u.q;
        tf2::Quaternion tf_q(output_q.x(),output_q.y(),output_q.z(),output_q.w());
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);// 四元数转欧拉角
        ROS_INFO("OUTPUTroll=%.2f, OUTPUTpitch=%.2f, OUTPUTyaw=%.2f", roll, pitch, yaw);
        posecmd.roll  = roll;  //x方向姿态(rad)
        posecmd.pitch = pitch;  //y方向姿态(rad)
        posecmd.yaw   = yaw;  //z方向姿态(rad)
        posecmd.throttle = u.thrust; //油门， （0.0-1.0）
        airsim_pub_.publish(posecmd);
    } 
}

void PX4CrtlNode::BodyrateCtrlPub(const CtrlOutput_t &u, const ros::Time &stamp)
{
    if (bd_.airsim_flag_) {

    } else {
        ROS_INFO("SET AIRSIM BODYRATE UNCESSFULLY!");
    //     mavros_msgs::AttitudeTarget msg;
    //     msg.header.stamp = stamp;
    //     msg.header.frame_id = std::string("FCU");
    //     msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    //     msg.body_rate.x = u.bodyrates.x();
    //     msg.body_rate.y = u.bodyrates.y();
    //     msg.body_rate.z = u.bodyrates.z();
    //     msg.thrust = u.thrust;
    //     px4ctrl_pub_.publish(msg);
    // }
    }
}
