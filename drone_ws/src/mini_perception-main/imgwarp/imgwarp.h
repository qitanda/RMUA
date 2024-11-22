#ifndef IMGWARP_H
#define IMGWARP_H

#include <vector>
#include <memory>
#include <string>
#include <future>
#include <opencv2/opencv.hpp>

namespace mini_perception {

class StereoWarpper {
public:
  StereoWarpper() = default;
  
  void Init(const cv::Mat &Kl, const cv::Mat &Dl, const cv::Mat &Rl, const cv::Mat &Pl,
            const cv::Mat &Kr, const cv::Mat &Dr, const cv::Mat &Rr, const cv::Mat &Pr,
            const int wl, const int hl, const int wr, const int hr);

  void Rectify(const cv::Mat &imgL, const cv::Mat &imgR, const cv::Mat &imgLr, const cv::Mat &imgRr);

private:
  cv::Mat K_;
  cv::Mat rmap_[2][2];
};

}

#endif // UNET_HPP