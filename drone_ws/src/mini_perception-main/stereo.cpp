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

#include <stereo/stereo.hpp>
#include <imgwarp/imgwarp.h>
using namespace std;

struct AffineMatrix {
  float i2d[6];       // image to dst(network), 2x3 matrix
  float d2i[6];       // dst to image, 2x3 matrix

  void compute(const cv::Size& from, const cv::Size& to, bool fix=true){
    float scale_x = to.width / (float)from.width;
    float scale_y = to.height / (float)from.height;
    float scale = std::min(scale_x, scale_y);

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

int main(){

  int deviceid = 0;
  TRT::set_device(deviceid);

  if(!iLogger::exists("trtstereo_fast.fp16.trtmodel")){
    printf("trtstereo_fast.fp16.trtmodel doesn't exist! Build the engine!\n");
    TRT::compile(TRT::Mode::FP16, 1, "trtstereo_fast.onnx", "trtstereo_fast.fp16.trtmodel");
  }

  const int eval_h = 160, eval_w = 320;
  // const std::string left_img_path = "/media/summervibe/5/dataset/DrivingStereo/left/2018-07-09-16-11-56";
  // const std::string right_img_path = "/media/summervibe/5/dataset/DrivingStereo/right/2018-07-09-16-11-56";

  // const std::string left_img_path = "/media/summervibe/5/dataset/DrivingStereo/left/2018-07-27-11-39-31";
  // const std::string right_img_path = "/media/summervibe/5/dataset/DrivingStereo/right/2018-07-27-11-39-31";
  // const std::string left_img_path = "/media/summervibe/5/dataset/apollo/ColorImage/Record043/Camera 5";
  // const std::string right_img_path = "/media/summervibe/5/dataset/apollo/ColorImage/Record043/Camera 6";
  const std::string left_img_path = "/media/summervibe/5/dataset/KAIST/urban39-pankyo_img/urban39-pankyo/image/stereo_left";
  const std::string right_img_path = "/media/summervibe/5/dataset/KAIST/urban39-pankyo_img/urban39-pankyo/image/stereo_right";
  const std::string config_leftcam_file = "/media/summervibe/5/dataset/KAIST/urban39-pankyo_calibration/urban39-pankyo/calibration/left.yaml";
  const std::string config_rightcam_file = "/media/summervibe/5/dataset/KAIST/urban39-pankyo_calibration/urban39-pankyo/calibration/right.yaml";
  
  // /media/summervibe/5/dataset/apollo/ColorImage/Record040/Camera 5
  assert(left_img_path == right_img_path);

  std::vector<std::string> left_names = ReadAllImages(left_img_path);
  std::vector<std::string> right_names = ReadAllImages(right_img_path);

  mini_perception::StereoNet stereo_net("trtstereo_fast.fp16.trtmodel");

  cv::Mat Kl, Dl, Rl, Pl, Kr, Dr, Rr, Pr;
  int wl, wr, hl, hr;

  cv::FileStorage fs;
  fs.open(config_leftcam_file, cv::FileStorage::READ);
  if(!fs.isOpened()) {
    INFOE("ERROR: The config file %s cannot been opened!", config_leftcam_file.c_str());
    return -1;
  }
  fs["camera_matrix"] >> Kl;
  fs["distortion_coefficients"] >> Dl;
  fs["rectification_matrix"] >> Rl;
  fs["projection_matrix"] >> Pl;
  fs["image_width"] >> wl;
  fs["image_height"] >> hl;
  fs.release();

  fs.open(config_rightcam_file, cv::FileStorage::READ);
  if(!fs.isOpened()) {
    INFOE("ERROR: The config file %s cannot been opened!", config_rightcam_file.c_str());
    return -1;
  }
  fs["camera_matrix"] >> Kr;
  fs["distortion_coefficients"] >> Dr;
  fs["rectification_matrix"] >> Rr;
  fs["projection_matrix"] >> Pr;
  fs["image_width"] >> wr;
  fs["image_height"] >> hr;
  fs.release();

  std::cout << "Kl: " << std::endl << Kl << std::endl;
  std::cout << "Dl: " << std::endl << Dl << std::endl;
  std::cout << "Rl: " << std::endl << Rl << std::endl;
  std::cout << "Pl: " << std::endl << Pl << std::endl;

  std::cout << "Kr: " << std::endl << Kr << std::endl;
  std::cout << "Dr: " << std::endl << Dr << std::endl;
  std::cout << "Rr: " << std::endl << Rr << std::endl;
  std::cout << "Pr: " << std::endl << Pr << std::endl;
  mini_perception::StereoWarpper warpper;
  warpper.Init(Kl, Dl, Rl, Pl, Kr, Dr, Rr, Pr, wl, hl, wr, hr);

  cv::Mat image_left, image_right;
  for(size_t i = 0; i < left_names.size(); ++i) {
    cv::Mat imgL = cv::imread(left_names[i], cv::IMREAD_UNCHANGED);
    cv::Mat imgR = cv::imread(right_names[i], cv::IMREAD_UNCHANGED);

    std::cout << "left name: " << left_names[i] << std::endl;
    std::cout << "right name: " << right_names[i] << std::endl;
    cv::cvtColor(imgL, imgL, cv::COLOR_BayerRG2BGR);  
		cv::cvtColor(imgR, imgR, cv::COLOR_BayerRG2BGR);
    cv::Mat disp_8u, disp_vis;

    cv::Mat imgLr(hl, wl, CV_8UC3);
    cv::Mat imgRr(hr, wr, CV_8UC3);
    std::cout << "Rectify!" << std::endl;
    double start = cv::getTickCount();
    warpper.Rectify(imgL, imgR, imgLr, imgRr);
    std::cout << "time cost: " << (cv::getTickCount() - start) / cv::getTickFrequency() << std::endl;
    
    // cv::resize(imgLr, imgLr, cv::Size(imgLr.cols/2, imgLr.rows/2), 0, 0, cv::INTER_LINEAR);
    // cv::resize(imgRr, imgRr, cv::Size(imgRr.cols/2, imgRr.rows/2), 0, 0, cv::INTER_LINEAR);
    cv::Mat hcon;
    cv::hconcat(imgLr, imgRr, hcon);

    for(int row = 10; row < hl; row += 20) {
      cv::line(hcon, cv::Point(0, row), cv::Point(wl + wr - 1, row), cv::Scalar(0, 255, 0), 1);
    }

    auto stereo_fut = stereo_net.Inference(imgLr, imgRr, true);
    cv::Mat disp = stereo_fut.get();
    disp = disp * 4;
    disp.convertTo(disp_8u, CV_8U);
    cv::applyColorMap(disp_8u, disp_vis, cv::COLORMAP_INFERNO);

    // cv::Mat disp_vis = stereo_net.get_disp_vis_img();

    cv::imshow("img", hcon);
    cv::imshow("disp", disp_vis);
    // while(1) {
    //   if(cv::waitKey(0) == 'q')
    //     break;
    // }
    cv::waitKey(1);    
  }


  return 0;
}