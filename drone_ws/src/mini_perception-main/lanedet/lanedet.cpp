#include "lanedet.hpp"
#include <atomic>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <trt_infer.hpp>
#include <ilogger.hpp>
#include <infer_controller.hpp>
#include <preprocess_kernel.cuh>
#include <monopoly_allocator.hpp>
#include <cuda_tools.hpp>

namespace mini_perception {

using namespace cv;
using namespace std;

cv::Mat LaneDet::WarpAffine(const cv::Mat& image, std::shared_ptr<TRT::Tensor>& tensor, int ibatch) {

  CUDAKernel::Norm normalize = CUDAKernel::Norm::alpha_beta(1.0f/255.0f, 0.0f, CUDAKernel::ChannelType::None);
  cv::Size input_size(tensor->size(3), tensor->size(2));
  printf("WarpAffine (%dx%d)->(%dx%d)\n", image.rows, image.cols, tensor->size(2), tensor->size(3));
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


void LaneDet::Inference(const cv::Mat& img) {
  const int height = img.rows, width = img.cols;

  cv::Mat imat = WarpAffine(img, input_, 0);
  
  double start = cv::getTickCount();
  std::cout << "start" << std::endl;
  engine_->forward(false);
  // auto output_ptr = output_->cpu<float>(0);
  std::cout << "output size: " << output_->size(2) << "x" << output_->size(3) << std::endl;
  printf("Forward Done. %lf s\n", (cv::getTickCount() - start) / cv::getTickFrequency());

  // cv::Mat res(2, 2, CV_32F, cv::Scalar(0));
  // std::promise<cv::Mat> pro;
  // pro.set_value(res);
  // return pro.get_future();
}

};