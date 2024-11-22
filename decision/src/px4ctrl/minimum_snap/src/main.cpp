#include <ros/ros.h>
#include "poly_planning.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh;

    PolyplanningClass poly_planning(nh);

    ros::spin();
    
    return 0;
}
