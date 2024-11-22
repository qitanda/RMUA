#include <ros/ros.h>
#include "px4ctrl.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4ctrl_node");
    ros::NodeHandle nh;

    PX4CrtlNode px4ctrl_node(nh);

    ros::spin();
    return 0;
}
