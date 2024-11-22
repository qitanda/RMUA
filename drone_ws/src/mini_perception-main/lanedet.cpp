#include <trt_builder.hpp>
#include <trt_infer.hpp>
#include <ilogger.hpp>
#include <lanedet/lanedet.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <thread>
#include <dirent.h>

using namespace std;

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

  if(!iLogger::exists("lanedet.fp32.trtmodel")){
    printf("lanedet.fp32.trtmodel doesn't exist! Build the engine!\n");
    TRT::compile(TRT::Mode::FP32, 1, "lanedet.onnx", "lanedet.fp32.trtmodel");
  }
  const std::string trtmodel_name = "lanedet.fp32.trtmodel";
  mini_perception::LaneDet::Ptr lane_det = mini_perception::LaneDet::CreateModel(trtmodel_name);

  cv::Mat img = cv::imread("./example.png", cv::IMREAD_UNCHANGED);
  // auto lane_fut = lane_det->Inference(img);
  lane_det->Inference(img);
  // cv::Mat res = lane_fut.get();

  return 0;
}