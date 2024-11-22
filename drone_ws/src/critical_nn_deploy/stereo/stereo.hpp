#ifndef STEREO_HPP
#define STEREO_HPP

#include <vector>
#include <memory>
#include <string>
#include <future>
#include <opencv2/opencv.hpp>
#include <trt_tensor.hpp>
#include <common/object_detector.hpp>

#include <tensorRT/preprocess_kernel.cuh>
#include <tensorRT/trt_tensor.hpp>
#include <tensorRT/trt_builder.hpp>
#include <tensorRT/trt_infer.hpp>
#include <tensorRT/ilogger.hpp>

namespace mini_perception {

class StereoNet {
public:
  StereoNet(const std::string& engine_file) {
    engine_ = TRT::load_infer(engine_file);
    if(engine_ == nullptr) {
      INFOE("Engine is nullptr");
      return;
    }
    input_left_ = engine_->tensor("bb_left");
    input_right_ = engine_->tensor("bb_right");  
    output_flow_ = engine_->tensor("rf4_flow_up");

    input_left_->resize_single_dim(0, 1).to_gpu();  
    input_right_->resize_single_dim(0, 1).to_gpu(); 

    eval_w_ = input_left_->size(3);
    eval_h_ = input_left_->size(2);
  }

  // ~StereoNet();

  struct AffineMatrix {
    float i2d[6];       // image to dst(network), 2x3 matrix
    float d2i[6];       // dst to image, 2x3 matrix

    void compute(const cv::Size& from, const cv::Size& to, bool fix=true) {
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

  static std::shared_ptr<StereoNet> CreateModel(const std::string& engine_file) {
    std::shared_ptr<StereoNet> net = std::make_shared<StereoNet>(engine_file);
    return net;
  }

  cv::Mat WarpAffine(const cv::Mat& image, std::shared_ptr<TRT::Tensor>& tensor, int ibatch);

  std::shared_future<cv::Mat> Inference(const cv::Mat& imgL, const cv::Mat& imgR, const bool vis);

  const cv::Mat& get_disp_vis_img() {
    return disp_vis_img_;
  }

  int get_eval_width() const {
    return eval_w_;
  }

  int get_eval_height() const {
    return eval_h_;
  }

private:
  std::shared_ptr<TRT::Infer> engine_;
  std::shared_ptr<TRT::Tensor> input_left_;
  std::shared_ptr<TRT::Tensor> input_right_;
  std::shared_ptr<TRT::Tensor> output_flow_;

  cv::Mat disp_img_;
  cv::Mat disp_vis_img_;
  int eval_w_, eval_h_;
};


}; // namespace UNet

#endif // UNET_HPP