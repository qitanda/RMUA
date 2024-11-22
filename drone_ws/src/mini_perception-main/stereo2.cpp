#include <tensorRT/preprocess_kernel.cuh>
#include <tensorRT/trt_tensor.hpp>
#include <tensorRT/trt_builder.hpp>
#include <tensorRT/trt_infer.hpp>
#include <tensorRT/ilogger.hpp>

#include <dirent.h>
#include <algorithm>
#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>

#include <yolo/yolo.hpp>
#include <unet/unet.hpp>

#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>        
#include <cv_bridge/cv_bridge.h>
#include "ros/time.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>//相近对齐
#include <message_filters/time_synchronizer.h>

using namespace std;

cv::Mat left_img, right_img;
double time1, time2;

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
  printf("Affine Transform [%dx%d]-->[%dx%d]\n", image.rows, image.cols, tensor->size(2), tensor->size(3));
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
  // sweep the directory
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

void callback(const sensor_msgs::ImageConstPtr &left_image,const  sensor_msgs::ImageConstPtr &right_image){    
    try
    {       
        cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::BGR8);
        left_img = image_ptr->image;     
        cv_bridge::CvImagePtr image_ptr1 = cv_bridge::toCvCopy(right_image, sensor_msgs::image_encodings::BGR8);
        right_img= image_ptr1->image; 
        time1 = (left_image -> header.stamp).toSec();
        time2 = (right_image -> header.stamp).toSec();
        cv::imshow("rgb_camera",left_img);
        cv::imshow("depth_camera",right_img);
        cv::waitKey(10);
    }
    catch(cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge Exception %s",e.what());
    }
}

int main(int argc, char  *argv[]){
  ros::init(argc,argv,"depth_calculate");
  ros::NodeHandle nh;
  // mySynchronizer wode;
  message_filters::Subscriber<sensor_msgs::Image> image_left_sub(nh, "/airsim_node/drone_1/front_left/Scene", 1, ros::TransportHints().tcpNoDelay());
  message_filters::Subscriber<sensor_msgs::Image> image_right_sub(nh, "/airsim_node/drone_1/front_right/Scene", 1, ros::TransportHints().tcpNoDelay()); 
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
  message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), image_left_sub, image_right_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  int deviceid = 0;
  TRT::set_device(deviceid);

  if(!iLogger::exists("trtstereo.fp16.trtmodel")){
    printf("trtstereo.fp16.trtmodel doesn't exist! Build the engine!\n");
    TRT::compile(TRT::Mode::FP16, 1, "../trtstereo.onnx", "trtstereo.fp16.trtmodel");
  }

  // return -1;

  std::shared_ptr<TRT::Infer> trtstereo = TRT::load_infer("trtstereo.fp16.trtmodel");
  if(trtstereo == nullptr) {
    INFOE("Engine is nullptr");
    return -1;
  }

  std::shared_ptr<TRT::Tensor> input_left = trtstereo->tensor("bb_left");
  std::shared_ptr<TRT::Tensor> input_right = trtstereo->tensor("bb_right");  
  std::shared_ptr<TRT::Tensor> output_flow = trtstereo->tensor("rf4_flow_up"); 

  const int eval_h = 480, eval_w = 640;
  const std::string left_img_path = "/home/rmuc3/test_img/left";
  const std::string right_img_path = "/home/rmuc3/test_img/right";
  assert(left_img_path == right_img_path);

  std::vector<std::string> left_names = ReadAllImages(left_img_path);
  std::vector<std::string> right_names = ReadAllImages(right_img_path);


  // cv::Mat image_left = cv::imread("left.jpg");
  // cv::Mat image_right = cv::imread("right.jpg");

  // if(image_left.empty() || image_right.empty()) {
  //   INFOE("Load image failed.");
  //   return -1;
  // }

  // use max = 1 batch to inference.
  int max_batch_size = 1;
  input_left->resize_single_dim(0, max_batch_size).to_gpu();  
  input_right->resize_single_dim(0, max_batch_size).to_gpu();

  int ibatch = 0;
  

  INFO("input_left.shape = %s", input_left->shape_string());
  INFO("input_right.shape = %s", input_right->shape_string());
  INFO("output_flow.shape = %s", output_flow->shape_string());

  // cv::Mat image_left, image_right;
  // for(size_t i = 0; i < left_names.size(); ++i) {
  //   cv::Mat imgL = cv::imread(left_names[i], cv::IMREAD_UNCHANGED);
  //   cv::Mat imgR = cv::imread(right_names[i], cv::IMREAD_UNCHANGED);   
  // }
  ros::spin();
  while (ros::ok())
  {
        if (!left_img.empty() && !right_img.empty() && time1 == time2)
        {
            double start = cv::getTickCount();
            cv::Mat imat_left = warpAffine(left_img, input_left, ibatch);
            cv::Mat imat_right = warpAffine(right_img, input_right, ibatch);
        
            trtstereo->forward(true);
            printf("Forward Done. %lf s\n", (cv::getTickCount() - start) / cv::getTickFrequency());

            cv::Mat disp(eval_h, eval_w, CV_32F);
            float *flow_data = output_flow->cpu<float>(ibatch);
            memcpy((float*)disp.data, flow_data, eval_h * eval_w * sizeof(float));
            disp = disp * 8;
            // disp = disp;
            cv::Mat disp_8u, disp_vis;
            disp.convertTo(disp_8u, CV_8U);
            cv::applyColorMap(disp_8u, disp_vis, cv::COLORMAP_JET);

            cv::imshow("img", disp_vis);
            cv::waitKey(10);    
        }
        ros::spinOnce();
  }
  return 0;
}