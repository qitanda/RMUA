#ifndef FEATURE_TRACKER_H
#define FEATURE_TRACKER_H
#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp> 
#include <opencv2/video/tracking.hpp>

#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "utility/tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
public:
  FeatureTracker();
  
  void detectGridFAST(const cv::Mat &img, const int ncellsize, 
      const std::vector<cv::Point2f> &vcurkps, std::vector<cv::Point2f>& kps);

  void trackMono(const cv::Mat &_img, double _cur_time);

  void setMask();

  void addPoints();

  bool updateID(unsigned int i);

  void readIntrinsicParameter(const string &calib_file);

  void showUndistortion(const string &name);

  void rejectWithF();

  void undistortedPoints();

  cv::Mat mask;
  cv::Mat fisheye_mask;
  cv::Mat prev_img, cur_img, forw_img;
  cv::Mat cur_depth;

  vector<cv::Point2f> n_pts;
  vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
  vector<cv::Point2f> prev_un_pts, cur_un_pts;
  vector<cv::Point2f> pts_velocity;
  vector<int> ids;
  vector<int> track_cnt;

  map<int, cv::Point2f> cur_un_pts_map;
  map<int, cv::Point2f> prev_un_pts_map;

  camodocal::CameraPtr m_camera;
  cv::Ptr<cv::FastFeatureDetector> fast_det;

  double cur_time;
  double prev_time;
  static int n_id;
};
#endif