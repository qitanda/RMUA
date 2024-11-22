#ifndef FIRST_DETECTION_H
#define FIRST_DETECTION_H

#include "ros/ros.h"
#include "AAMED/src/FLED.h"
#include <opencv2/opencv.hpp>     
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>

class FirstDetector
{
    public :
        FirstDetector();
        // FirstDetector(ros::NodeHandle *nh,bool debug1);
        ~FirstDetector();
        AAMED *aamed = new AAMED(480,640);

        void run(const cv::Mat);
        void drawresult_max(cv::Mat &image, const std::vector<cv::RotatedRect> detcircles);
        void drawresult_common(cv::Mat &image,const std::vector<cv::RotatedRect> detCircles);
        std::vector<cv::Point2f> ellipse_extract(const cv::Mat disparity, std::vector<cv::RotatedRect> detCircles);
        std::vector<cv::Point2f> single_ellipse_extract(const cv::Mat disparity, std::vector<cv::RotatedRect> detCircles);
        cv::Point2f single_circle_center(std::vector<cv::RotatedRect> detCircles);

    private:
        cv::Mat leftimage,rightimage;
        // ros::NodeHandle first_nh;
        bool debug;
		double A1, B1, C1, D1, E1, F1;
		double A2, B2, C2, D2, E2, F2;
        void _pre_lab(const cv::Mat inimage, cv::Mat &outimage,bool detect_blue);
        void _filter_result_max(std::vector<cv::RotatedRect> incircles,cv::RotatedRect &outcircle);
        void _ellipse_parameters(const std::vector<cv::RotatedRect> detCircles);

};

#endif