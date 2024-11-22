
#include <math.h>
#include <boost/thread/thread.hpp>
#include <fstream>
   // PCL specific includes

#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/conditional_removal.h>
using namespace std;

int main(int argc, char ** argv)
{
    Eigen::Matrix3d Rz,Ry,Rx,R_switchy;
    Ry<< cos(M_PI/2),0,sin(M_PI/2),
         0,1,0,
         -sin(M_PI/2),0,cos(M_PI/2);
//     double pitch = 0.5*M_PI;
//     Rx<< 1,0,0,
//          0,cos(pitch),-sin(pitch),
//          0,sin(pitch),cos(pitch);
    double yaw =  0.5*M_PI;

    Rz<< cos(yaw),-sin(yaw),0,
         sin(yaw),cos(yaw),0,
         0,0,1;
   
     // R_switchy<<1,0,0,
     //           0,-1,0,
     //           0,0, 1;
   Eigen::Matrix3d R_mat = Rz*Ry;
   std::cout<<R_mat<<std::endl;
   std::cout<<R_mat.inverse()<<std::endl;

}
    
