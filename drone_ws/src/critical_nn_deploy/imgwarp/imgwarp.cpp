#include "imgwarp.h"

namespace mini_perception {

using namespace cv;

void StereoWarpper::Init(const cv::Mat &Kl, const cv::Mat &Dl, const cv::Mat &Rl, const cv::Mat &Pl,
            const cv::Mat &Kr, const cv::Mat &Dr, const cv::Mat &Rr, const cv::Mat &Pr,
            const int wl, const int hl, const int wr, const int hr) {
  // cv::initUndistortRectifyMap(Kl, Dl, Rl, Pl.rowRange(0,3).colRange(0,3), cv::Size(wl, hl), CV_16SC2, rmap_[0][0], rmap_[0][1]);
  // cv::initUndistortRectifyMap(Kr, Dr, Rr, Pr.rowRange(0,3).colRange(0,3), cv::Size(wr, hr), CV_16SC2, rmap_[1][0], rmap_[1][1]);
  cv::initUndistortRectifyMap(Kl, Dl, Rl, Pl, cv::Size(640, 320), CV_16SC2, rmap_[0][0], rmap_[0][1]);
  cv::initUndistortRectifyMap(Kr, Dr, Rr, Pr, cv::Size(640, 320), CV_16SC2, rmap_[1][0], rmap_[1][1]);

}

void StereoWarpper::Rectify(const cv::Mat &imgL, const cv::Mat &imgR, const cv::Mat &imgLr, const cv::Mat &imgRr) {
  // std::cout << rmap_[0][0].size() << std::endl;
  // std::cout << imgL.size() << std::endl;
  // std::cout << imgR.size() << std::endl;
  // std::cout << rmap_[1][1].size() << std::endl;
  cv::remap(imgL, imgLr, rmap_[0][0], rmap_[0][1], cv::INTER_LINEAR);
  cv::remap(imgR, imgRr, rmap_[1][0], rmap_[1][1], cv::INTER_LINEAR);
}

}