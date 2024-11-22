#include "path_generator.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "waypoint_node");
    ros::NodeHandle nh;
    WayPointGenerator PointGentor(nh);
    PointGentor.GenerateWaypoints();
    //std::cout<<"GenerateWayPoints!"<<std::endl;
    PointGentor.OutputAllWayPoints();
    //std::cout<<"OutputAllWayPoints!"<<std::endl;
    ros::Rate(30);
    ros::Duration(1).sleep();
    bool pub_points = false;
  
    while(ros::ok())
    {
        ros::spinOnce();
        //PointGentor.PublishWayPoints();
        if(!pub_points)
        {
            pub_points = true;
            PointGentor.PublishWayPoints();
        }
    }
    return 0;

}

