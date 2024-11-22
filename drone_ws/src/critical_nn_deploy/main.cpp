#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "ros/time.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h> 
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <tensorRT/preprocess_kernel.cuh>
#include <tensorRT/trt_tensor.hpp>
#include <tensorRT/trt_builder.hpp>
#include <tensorRT/trt_infer.hpp>
#include <tensorRT/ilogger.hpp>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <dirent.h>
#include <algorithm>
#include <thread>
#include "pcl_handle.hpp"
#include "circle_detection/first_detection.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include <AAMED/src/FLED.h>
#include <yolo/yolo.hpp>
#include <unet/unet.hpp>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "airsim_ros/Circle.h"
#include "airsim_ros/CirclePoses.h"
#include "std_msgs/Int32.h"
#include "deque"
#include "kalman_filter.h"

//kalmanfilter 参数（需要调整）
float measure_noise=0.1;
float process_noise=0.1;
KalmanCV KF_x,KF_y,KF_z,KF_yaw; //四个一维代表三个坐标,加上一个yaw角
double last_time,now_time,dt,aver_yaw;
bool first_flag = true;


// #include "flag_msgs/mode_flag.h"

using namespace std;

cv::Mat imgL, imgR;
std::shared_ptr<TRT::Infer> trtstereo;
std::shared_ptr<TRT::Tensor> input_left;
std::shared_ptr<TRT::Tensor> input_right;
std::shared_ptr<TRT::Tensor> output_flow;
int last_target_id = 0, target_id = 0, mode = 3;
// Only for debug
ros::Publisher pc_pub, odom_pub, circle_pub, circlegt_pub, depth_pub;
pcl_handle::depth_extract depth_extract_;
FirstDetector Left;
Eigen::Quaterniond Qti, Qwiwt;
Eigen::Vector3d tti;
bool odom_initialize = 0, circle_get = 0, tensorrt_flag = 0;
airsim_ros::Circle circle_gt[16];
// AAMED aamed(540, 960);
// aamed.SetParameters(CV_PI / 3, 3.4, 0.77); // config parameters of AAMED
class OdomData
{
  public:
    double time = 0.0;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    double yaw;
};
std::deque<OdomData> odomBuff;
std::deque<OdomData> new_odom_data_;
std::deque<geometry_msgs::Vector3> target_pos;
std::deque<double> target_yaw;
void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg_ptr)
{
    if (odom_initialize == 0) 
    {
      odom_initialize = 1;
    }
    OdomData odom_data;
    std::cout<<"!!!!"<<std::endl;
    odom_data.time = odom_msg_ptr->header.stamp.toSec();
    Eigen::Matrix4d translation_matrix = Eigen::Matrix4d::Identity();
    Eigen::Vector3d twtt(odom_msg_ptr->pose.pose.position.x, odom_msg_ptr->pose.pose.position.y, odom_msg_ptr->pose.pose.position.z);

    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = twtt.x();
    odom.pose.pose.position.y = twtt.y();
    odom.pose.pose.position.z = twtt.z();
    odom.pose.pose.orientation.w = odom_msg_ptr->pose.pose.orientation.w;
    odom.pose.pose.orientation.x = odom_msg_ptr->pose.pose.orientation.x;
    odom.pose.pose.orientation.y = odom_msg_ptr->pose.pose.orientation.y;
    odom.pose.pose.orientation.z = odom_msg_ptr->pose.pose.orientation.z;
    Eigen::Quaterniond Q(odom_msg_ptr->pose.pose.orientation.x,
                            odom_msg_ptr->pose.pose.orientation.y, odom_msg_ptr->pose.pose.orientation.z,odom_msg_ptr->pose.pose.orientation.w);
    odom_data.R = Q.matrix();
    odom_data.t = twtt;
    odom_data.yaw = odom_msg_ptr->pose.pose.orientation.z;
    odom.header.frame_id = "map";
    new_odom_data_.push_back(odom_data);
    odom_pub.publish(odom);
}

void circlegtCallback(const airsim_ros::CirclePosesConstPtr &circle)
{
    if (circle_get == 0)
    {
      for (int i = 0; i < 16; ++i)
      {
        circle_gt[i] = circle->poses[i];
      }
      circle_get = 1;
    }
}

void odomgtCallback(const geometry_msgs::PoseStampedConstPtr &odom_msg_ptr)
{
    OdomData odom_data;
    odom_data.time = odom_msg_ptr->header.stamp.toSec();
    Eigen::Matrix4d translation_matrix = Eigen::Matrix4d::Identity();
    Eigen::Vector3d twtt(odom_msg_ptr->pose.position.x, odom_msg_ptr->pose.position.y, odom_msg_ptr->pose.position.z);
    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = twtt.x();
    odom.pose.pose.position.y = twtt.y();
    odom.pose.pose.position.z = twtt.z();
    odom.pose.pose.orientation.w = odom_msg_ptr->pose.orientation.w;
    odom.pose.pose.orientation.x = odom_msg_ptr->pose.orientation.x;
    odom.pose.pose.orientation.y = odom_msg_ptr->pose.orientation.y;
    odom.pose.pose.orientation.z = odom_msg_ptr->pose.orientation.z;
    Eigen::Quaterniond Q(odom_msg_ptr->pose.orientation.w, odom_msg_ptr->pose.orientation.x,
                            odom_msg_ptr->pose.orientation.y, odom_msg_ptr->pose.orientation.z);
    odom_data.R = Q.matrix();
    odom_data.t = twtt;
    odom_data.yaw = odom_msg_ptr->pose.orientation.z;
    odom.header.frame_id = "map";
    new_odom_data_.push_back(odom_data);
    odom_pub.publish(odom);
}

void ParseData(std::deque<OdomData> &odom_data_buff)
{
    if (new_odom_data_.size() > 0)
    {
        odom_data_buff.insert(odom_data_buff.end(), new_odom_data_.begin(), new_odom_data_.end());
        new_odom_data_.clear();
    }
}

void findNearestOdom(std::deque<OdomData> &odomBuff, OdomData &odom, double img_time)
{
    while (odomBuff.size() > 0)
    {
        OdomData odom_data = odomBuff.front();
        double delta = img_time - odom_data.time;
        if (delta < -0.01)
            break;
        else if (delta > 0.01)
            odomBuff.pop_front();
        else
        {
            odom = odom_data;
            odomBuff.pop_front();
            break;
        }
    }
}

geometry_msgs::Vector3 getworldpose(OdomData odom, Eigen::Vector3d relative_coordinates) {
    Eigen::Matrix4d translation_matrix = Eigen::Matrix4d::Identity();

    translation_matrix.block<3, 1>(0, 3) = odom.t;
    translation_matrix.block<3, 3>(0, 0) = odom.R;
    Eigen::Vector4d world_coordinates = translation_matrix * Eigen::Vector4d(relative_coordinates.x(), relative_coordinates.y(), relative_coordinates.z(), 1.0);//--
    // Eigen::Vector3d world_coordinates = pose * Eigen::Vector3d(relative_coordinates.x(), relative_coordinates.y(), relative_coordinates.z());//--++
    Eigen::Vector3d final_world_coordinates = world_coordinates.head(3);
    // std::cout << "T: " << translation_matrix << std::endl;
    //std::cout << "世界坐标: " << final_world_coordinates << std::endl;
    geometry_msgs::Vector3 g;
    g.x = final_world_coordinates[0];
    g.y = final_world_coordinates[1];
    g.z = final_world_coordinates[2];
    return g;
}

void targetidCallback(const std_msgs::Int32ConstPtr &id_msg)
{
  target_id = id_msg->data;
}

void modeCallback(const std_msgs::Int32ConstPtr &mode_msg)
{
  mode = mode_msg->data;
}

struct AffineMatrix{
  float i2d[6];       // image to dst(network), 2x3 matrix
  float d2i[6];       // dst to image, 2x3 matrix

  void compute(const cv::Size& from, const cv::Size& to, bool fix=true){
    float scale_x = to.width / (float)from.width;
    float scale_y = to.height / (float)from.height;
    float scale = std::min(scale_x, scale_y);
    /* 
            + scale * 0.5 - 0.5 的主要原因是使得中心更加对齐，下采样不明显，但是上采样时就比较明显
        参考：https://www.iteye.com/blog/handspeaker-1545126
    */
    if(fix) {
      i2d[0] = scale;  i2d[1] = 0;  i2d[2] = -scale * from.width  * 0.5  + to.width * 0.5 + scale * 0.5 - 0.5;
      i2d[3] = 0;  i2d[4] = scale;  i2d[5] = -scale * from.height * 0.5 + to.height * 0.5 + scale * 0.5 - 0.5;
    }
    else {
      i2d[0] = scale_x;  i2d[1] = 0;  i2d[2] = -scale_x * from.width  * 0.5  + to.width * 0.5 + scale_x * 0.5 - 0.5;
      i2d[3] = 0;  i2d[4] = scale_y;  i2d[5] = -scale_y * from.height * 0.5 + to.height * 0.5 + scale_y * 0.5 - 0.5;
    }
    
    cv::Mat m2x3_i2d(2, 3, CV_32F, i2d);
    cv::Mat m2x3_d2i(2, 3, CV_32F, d2i);
    cv::invertAffineTransform(m2x3_i2d, m2x3_d2i);
  }

  cv::Mat i2d_mat(){
    return cv::Mat(2, 3, CV_32F, i2d);
  }

  cv::Mat d2i_mat(){
    return cv::Mat(2, 3, CV_32F, d2i);
  }
};

static cv::Mat warpAffine(const cv::Mat& image, shared_ptr<TRT::Tensor>& tensor, int ibatch){

  CUDAKernel::Norm normalize = CUDAKernel::Norm::alpha_beta(2.0f/255.0f, -1.0f, CUDAKernel::ChannelType::None);
  cv::Size input_size(tensor->size(3), tensor->size(2));

  AffineMatrix affine;
  affine.compute(image.size(), input_size, false);

  size_t size_image      = image.cols * image.rows * 3;
  size_t size_matrix     = iLogger::upbound(sizeof(affine.d2i), 32);
  auto workspace         = tensor->get_workspace();
  uint8_t* gpu_workspace        = (uint8_t*)workspace->gpu(size_matrix + size_image);
  float*   affine_matrix_device = (float*)gpu_workspace;
  uint8_t* image_device         = size_matrix + gpu_workspace;

  uint8_t* cpu_workspace        = (uint8_t*)workspace->cpu(size_matrix + size_image);
  float* affine_matrix_host     = (float*)cpu_workspace;
  uint8_t* image_host           = size_matrix + cpu_workspace;
  auto stream                   = tensor->get_stream();

  memcpy(image_host, image.data, size_image);
  memcpy(affine_matrix_host, affine.d2i, sizeof(affine.d2i));
  checkCudaRuntime(cudaMemcpyAsync(image_device, image_host, size_image, cudaMemcpyHostToDevice, stream));
  checkCudaRuntime(cudaMemcpyAsync(affine_matrix_device, affine_matrix_host, sizeof(affine.d2i), cudaMemcpyHostToDevice, stream));

  CUDAKernel::warp_affine_bilinear_and_normalize_plane(
    image_device,               image.cols * 3,       image.cols,       image.rows, 
    tensor->gpu<float>(ibatch), input_size.width,     input_size.height, 
    affine_matrix_device, 128, 
    normalize, stream
  );
  return affine.d2i_mat().clone();
}

std::vector<std::string> ReadAllImages(const std::string& img_dir_path)
{
  std::vector<std::string> filenames;

  DIR* dir;
  if ((dir = opendir(img_dir_path.c_str())) == nullptr) {
    throw std::runtime_error("directory " + img_dir_path + " does not exist");
  }
  dirent* dp;
  for (dp = readdir(dir); dp != nullptr; dp = readdir(dir)) {
    const std::string img_file_name = dp->d_name;
    if (img_file_name == "." || img_file_name == "..") {
      continue;
    }
    filenames.push_back(img_dir_path + "/" + img_file_name);
  }
  closedir(dir);

  std::sort(filenames.begin(), filenames.end());
  return filenames;
}

void depth_calculate(cv::Mat disparity, std::vector<cv::Point2f> point_set, cv::Point2f circle_img_center, double img_time) {
    ParseData(odomBuff);
    OdomData odom, odom_circle;
    findNearestOdom(odomBuff, odom, img_time);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_ptr->width = point_set.size();
    cloud_ptr->height = 1;
    cloud_ptr->is_dense = false;                      
    cloud_ptr->points.resize(cloud_ptr->width); 
    double fx = 320.0,fy=320.0,cx=320.0,cy=240.0;
    double b = 0.095;
    double baseline = 95;
    int count = 0;
    cv::Point3f circle(0, 0, 0);
    for (size_t i = 0; i < point_set.size(); ++i)
    {
      double x = (point_set[i].x - cx) / fx;
      double y = (point_set[i].y - cy) / fy;
      double depth = fx * baseline / (disparity.at<float>(point_set[i].y, point_set[i].x));
      cloud_ptr->points[i] = pcl::PointXYZ(x * depth / 1000, y * depth / 1000, depth / 1000);
    }
    // pcl::PointXYZ min_point, max_point;
    if (cloud_ptr->points.size() == 0) return;
    filter_cloud_ptr = depth_extract_.outlier_filter(cloud_ptr);
    if (filter_cloud_ptr->points.size() <= 50) return;


    for (size_t i = 0; i < filter_cloud_ptr->points.size(); ++i)
    {
      circle.x += filter_cloud_ptr->points[i].x / filter_cloud_ptr->points.size();
      circle.y += filter_cloud_ptr->points[i].y / filter_cloud_ptr->points.size();
      circle.z += filter_cloud_ptr->points[i].z / filter_cloud_ptr->points.size();
    }
    odom_circle = odom;
    depth_extract_.PCAPlanFitting(filter_cloud_ptr, odom_circle.yaw);
    circle.x = (circle_img_center.y - cx) / fx * circle.z;
    circle.y = (circle_img_center.x - cy) / fy * circle.z;
    circle.x -= 0.0475;
    circle.z += 0.26;


    sensor_msgs::PointCloud2 debug_pc;
    pcl::toROSMsg(*filter_cloud_ptr,debug_pc);
    debug_pc.header.frame_id = "map";
    debug_pc.header.stamp = ros::Time::now();
    pc_pub.publish(debug_pc);


    geometry_msgs::Vector3 ordinate = getworldpose(odom, Eigen::Vector3d(circle.z, circle.x, circle.y));//z,x,y
    if (last_target_id != target_id)
    {
      last_target_id = target_id;
      first_flag = true;
      target_pos.clear();
      target_yaw.clear();
    }
    std::cout<<"target ID :" <<target_id<<std::endl;
    std::cout<<sqrt(pow(ordinate.x - circle_gt[target_id].position.x, 2) + pow(ordinate.y - circle_gt[target_id].position.y, 2) + pow(ordinate.z - circle_gt[target_id].position.z, 2))<<std::endl;
    if (sqrt(pow(ordinate.x - circle_gt[target_id].position.x, 2) + pow(ordinate.y - circle_gt[target_id].position.y, 2) + pow(ordinate.z - circle_gt[target_id].position.z, 2) < 25))
    {
      double dis = sqrt(pow(circle.z, 2) + pow(circle.x, 2) + pow(circle.y, 2));
      if (dis < 15 && dis >2 )
      {
        geometry_msgs::Vector3 pos, aver_pos;
        
        airsim_ros::Circle p;
        pos = ordinate;
        aver_yaw = odom_circle.yaw;
        target_pos.push_back(pos);
        // target_yaw.push_back(aver_yaw);
        if (target_pos.size() > 5) 
        {
          target_pos.pop_front();
          // target_yaw.pop_front();
        }
        aver_pos.x = 0;
        aver_pos.y = 0;
        aver_pos.z = 0;
        aver_yaw =0;
        for (int i = 0; i < target_pos.size(); ++i)
        {
          aver_pos.x += target_pos[i].x / target_pos.size();
          aver_pos.y += target_pos[i].y / target_pos.size();
          aver_pos.z += target_pos[i].z / target_pos.size();
          // aver_yaw +=target_yaw[i]/target_yaw.size();
        }

        // 以下为卡尔曼滤波
         now_time= ros::Time::now().toSec();
        if(first_flag)//传了不同flag需要重置
        {
            first_flag=false;
            Eigen::Vector2d Px(aver_pos.x,0);
            Eigen::Vector2d Py(aver_pos.y,0);
            Eigen::Vector2d Pz(aver_pos.z,0);
            Eigen::Vector2d Yaw(aver_yaw,0);
            KF_x.Reset(Px);
            KF_y.Reset(Py);
            KF_z.Reset(Pz);
            // KF_yaw.Reset(Yaw);
        }
        else
        {
          dt =  now_time-last_time;
          KF_x.MeasureUpdate(aver_pos.x,dt); //dt为时间戳差值 ,需要写一个时间戳
          KF_y.MeasureUpdate(aver_pos.y,dt);
          KF_z.MeasureUpdate(aver_pos.z,dt);
          // KF_yaw.MeasureUpdate(aver_yaw,dt);
          std::cout<<"dt"<<std::endl;
        }
        // 传出预测后的值。
        aver_pos.x=KF_x.X_(0);
        aver_pos.y=KF_y.X_(0);
        aver_pos.z=KF_z.X_(0);
        // aver_yaw = KF_yaw.X_(0);
        target_pos.push_back(pos);
        // target_yaw.push_back(aver_yaw);
        p.index = 0;
        p.position = aver_pos;
        p.yaw = odom_circle.yaw;
        if (circle.z < 7) p.yaw = odom_circle.yaw / M_PI * 180;
        else p.yaw = 0;
        last_time = now_time;
        circle_pub.publish(p);
        std::cout << "世界坐标: " << aver_pos << std::endl;
      }
    }

}

void depth_calculate1(cv::Mat disparity, double img_time) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_ptr->width = disparity.rows * disparity.cols;
    cloud_ptr->height = 1;
    cloud_ptr->is_dense = false;                      
    cloud_ptr->points.resize(cloud_ptr->width); 
    double fx = 320.0,fy=320.0,cx=320.0,cy=240.0;
    double b = 0.095;
    double baseline = 95;
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);
    int count = 0;
    cv::Point3f circle(0, 0, 0);
    for (size_t i = 0; i < disparity.cols; ++i)
    {
      for (size_t j = 0; j < disparity.rows; ++j)
      {
      double x = (i - cx) / fx;
      double y = (j - cy) / fy;
      double depth = fx * baseline / (disparity.at<float>(i, j));
      cloud_ptr->points[i] = pcl::PointXYZ(x * depth / 1000, y * depth / 1000, depth / 1000);
      }
    }
    pcl::PointXYZ min_point, max_point;
    // depth_extract_.normal_estimate(cloud_ptr);
    filter_cloud_ptr = depth_extract_.outlier_filter(cloud_ptr);
    if (filter_cloud_ptr->points.size() <= 50) return;
    sensor_msgs::PointCloud2 debug_pc;
    pcl::toROSMsg(*filter_cloud_ptr,debug_pc);
    debug_pc.header.frame_id = "map";
    debug_pc.header.stamp = ros::Time::now();
    pc_pub.publish(debug_pc);
}

cv::Mat disptodepth(const cv::Mat &src)
{
    double fx = 320.0,fy=320.0,cx=320.0,cy=240.0;
    double b = 0.095;
    double baseline = 95;
    cv::Mat depth(src.rows, src.cols, CV_16UC1, cv::Scalar(0));
    for (size_t i = 0; i < src.cols; ++i)
    {
      for (size_t j = 0; j < src.rows; ++j)
      {

        depth.at<ushort>(j, i) = std::min(6e4, std::round((fx * b / (src.at<float>(j, i))) * 1000));
      }
    }
    return depth;
}

void callback(const sensor_msgs::ImageConstPtr &left_image, const sensor_msgs::ImageConstPtr &right_image){    
  try {       
    cv_bridge::CvImagePtr left_ptr = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::BGR8);
    imgL = left_ptr->image;     
    cv_bridge::CvImagePtr right_ptr = cv_bridge::toCvCopy(right_image, sensor_msgs::image_encodings::BGR8);
    imgR = right_ptr->image; 
    double time1 = (left_image -> header.stamp).toSec();
    double time2 = (right_image -> header.stamp).toSec();
    // if (time1 != time2) std::cout<< "aaaaaaaa"<<std::endl;
    double start = cv::getTickCount();
    cv::Mat imat_left = warpAffine(imgL, input_left, 0);
    cv::Mat imat_right = warpAffine(imgR, input_right, 0);

    trtstereo->forward(true);
    //printf("Forward Done. %lf s\n", (cv::getTickCount() - start) / cv::getTickFrequency());

    cv::Mat disp(480, 640, CV_32F);
    cv::Mat imgLG;
    float *flow_data = output_flow->cpu<float>(0);
    memcpy((float*)disp.data, flow_data, 480 * 640 * sizeof(float));

    auto depth = disptodepth(disp);
     // 将OpenCV的图像转换为ROS的图像消息
    cv_bridge::CvImage cv_img;
    cv_img.header.stamp = left_image -> header.stamp;
    cv_img.header.frame_id = "camera_frame";
    cv_img.encoding = "mono16";  // 图像编码为uint16类型
    cv_img.image = depth;

  // 发布图像消息
  // image_pub.publish(cv_img.toImageMsg());
  //   cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::MONO16).toImageMsg();
  //   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),sensor_msgs::image_encodings::MONO16,depth);
    
    depth_pub.publish(cv_img.toImageMsg());

    // depth_calculate(disp);
    disp = disp * 2;//disp*2是为了可视化，视差真值是*2之前的值
    cv::Mat disp_8u, disp_vis;
    disp.convertTo(disp_8u, CV_8U);
    cv::Mat blankImage1(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
    Left.run(imgL, disp, mode) ;// run AAMED
    Left.drawresult_common(blankImage1,Left.aamed->detEllipses);
    std::vector<cv::Point2f> point_set = Left.single_ellipse_extract(disp / 2, Left.resultCircles);
    cv::Point2f circle_img_center = Left.single_circle_center(Left.resultCircles);
    // std::cout<<"POINT SET SIZE"<<point_set.size()<<std::endl;
    if (!point_set.empty()) depth_calculate(disp / 2, point_set, circle_img_center, time1);
    cv::applyColorMap(disp_8u, disp_vis, cv::COLORMAP_JET);
    // cv::imshow("rgb", imgL);
    // cv::imshow("img", disp_vis);
    // // cv::imshow("left",blankImage1);
    // cv::waitKey(1);    
  }
  catch(cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge Exception %s",e.what());
  }
}

int main(int argc,char **argv) {

  int deviceid = 0;
  if (tensorrt_flag == 0) 
  {
    TRT::set_device(deviceid);
    trtstereo = TRT::load_infer("/home/criticalhit/trtstereo.fp16.trtmodel");
    if(trtstereo == nullptr) {
      INFOE("Engine is nullptr");
      return -1;
    }
    Eigen::Matrix3d Ric, Rct, Rwiwt;
    tti << -0.26, 0.0475, 0;
    Rwiwt = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * 
              Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());
    Qwiwt = Eigen::Quaterniond(Rwiwt);
    tensorrt_flag = 1;
  }
  ros::init(argc, argv, "perception_node");
  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image> image_left_sub(nh, "/airsim_node/drone_1/front_left/Scene", 1, ros::TransportHints().tcpNoDelay());
  message_filters::Subscriber<sensor_msgs::Image> image_right_sub(nh, "/airsim_node/drone_1/front_right/Scene", 1, ros::TransportHints().tcpNoDelay()); 
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
  pc_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud",1);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom",1);
  depth_pub = nh.advertise<sensor_msgs::Image>("depth",1);
  circle_pub = nh.advertise<airsim_ros::Circle>("/img_circle",1);
  circlegt_pub = nh.advertise<airsim_ros::Circle>("/circle_gt_first",1);
  message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), image_left_sub, image_right_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  // ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/imu_propagate", 100, odomCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber odom_sub = nh.subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/debug/pose_gt", 100, odomgtCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber circlegt_sub = nh.subscribe<airsim_ros::CirclePoses>("/airsim_node/drone_1/circle_poses", 100, circlegtCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber count_sub = nh.subscribe<std_msgs::Int32>("target_ID", 10, targetidCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber mode_sub = nh.subscribe<std_msgs::Int32>("mode_flag", 10, modeCallback, ros::TransportHints().tcpNoDelay());


  input_left = trtstereo->tensor("bb_left");
  input_right = trtstereo->tensor("bb_right");  
  output_flow = trtstereo->tensor("rf4_flow_up"); 

    //卡尔曼滤波初始化代码
    KF_x.Init(0.0,process_noise,measure_noise);
    KF_y.Init(0.0,process_noise,measure_noise);
    KF_z.Init(0.0,process_noise,measure_noise);
    KF_yaw.Init(0.0,process_noise,measure_noise);
  // // use max = 1 batch to inference.
  int max_batch_size = 1;
  input_left->resize_single_dim(0, max_batch_size).to_gpu();  
  input_right->resize_single_dim(0, max_batch_size).to_gpu();
  
  ros::spin();
  return 0;
}