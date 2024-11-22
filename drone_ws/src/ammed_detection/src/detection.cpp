#include "first_detection.h"
#include <ros/ros.h>
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


cv::Mat left_img,right_img;
cv::Mat imgL,imgR;
int filter_result_max(std::vector<cv::RotatedRect> incircles,cv::RotatedRect &outcircle);

void image_left_callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImageConstPtr cv_image;
	cv_image = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	left_img = cv_image->image;
	//std::cout<<left_img.cols<<left_img.rows<<std::endl;

	cv::imshow("left_image",left_img);
	cv::waitKey(10);
	
}

void image_right_callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImageConstPtr cv_image;
	cv_image = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	right_img = cv_image->image;
	// cv::imshow("right_image",right_img);
	// cv::waitKey(10);
}

void pre_lab(const cv::Mat inimage, cv::Mat &outimage,bool detect_blue)
{
	cv::Mat blurimg,blur1_img,blur2_img,Labimg, thrimg0,
		thrimg,thrimg2,thrimg3,eroimg,diaimg,blur3_img,blur4_img, 
		blue_L, blue_b,blue,blue_erode,blue_dilate,blue_blur;
	//以下是图像预处理
	// cv::medianBlur(inimage,blur1_img,3);
	// cv::medianBlur(blur1_img,blur2_img,3);
	// cv::cvtColor(blur2_img,Labimg,cv::COLOR_BGR2Lab);
	cv::cvtColor(inimage,Labimg,cv::COLOR_BGR2Lab);
	std::vector<cv::Mat> lab;
	cv::split(Labimg,lab);
	cv::threshold(lab[1],thrimg0,155,255,CV_THRESH_BINARY);
	cv::medianBlur(thrimg0,thrimg0, 3);
	cv::threshold(lab[2],thrimg2,180,255,CV_THRESH_BINARY_INV);
	cv::threshold(lab[2],thrimg3,70,255,CV_THRESH_BINARY);
	thrimg=thrimg0&(thrimg2&thrimg3);
    cv::Mat erodeKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	// cv::erode(thrimg,eroimg,erodeKernel);
	cv::Mat dilateKernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
	// cv::dilate(eroimg,diaimg,dilateKernel);
	// cv::medianBlur(diaimg,blur3_img, 3);
	cv::medianBlur(thrimg,blur3_img,3);
	// blurimg = blur3_img;
	//加入双边滤波
	cv::bilateralFilter(blur3_img,blur4_img,5,100,100);
	blurimg = blur4_img;
	//以下是识别蓝色
	cv::threshold(lab[0], blue_L, 70, 255, CV_THRESH_BINARY);
    cv::threshold(lab[2], blue_b, 110, 255, CV_THRESH_BINARY_INV);
    blue = blue_L & blue_b;
    cv::erode(blue, blue_erode, erodeKernel); //执行腐蚀操作
    cv::dilate(blue_erode, blue_dilate, dilateKernel);

    cv::medianBlur(blue_dilate, blue_blur,3);
    if (detect_blue ==true)
    {
        outimage = blurimg | blue_blur;
    }
    else
    {
        outimage = blurimg;
    }

	//debug 展示功能
cv::imshow("thrimg0",thrimg0);
cv::imshow("thrimg",thrimg);
cv::imshow("blurimg",blurimg);
	cv::waitKey(5);
}

//直接取最大圆环
int filter_result_max(std::vector<cv::RotatedRect> incircles,cv::RotatedRect &outcircle)
{
	float max=0;
	int index=-1;
	for ( int i=0;i<incircles.size();i++)
	{
		if((incircles[i].size.height+incircles[i].size.width) > max)
			{
				max = incircles[i].size.height+incircles[i].size.width;
				index = i;
			}

	}
	if(index!=-1)
	{
        // if((rect.size.height == 0 || rect.size.width == 0)||((incircles[index].center.y - rect.center.y)* (incircles[index].center.y - rect.center.y)+(incircles[index].center.x - rect.center.x)* (incircles[index].center.x - rect.center.x))<5000)
        // {
            outcircle.center.x = incircles[index].center.y;
            outcircle.center.y = incircles[index].center.x;
            outcircle.size.height = incircles[index].size.width;
            outcircle.size.width = incircles[index].size.height;
            outcircle.angle = -incircles[index].angle;
            // rect = outcircle;
            return 1;
        // }
        // return 0;
	}
    return 0;
}

void drawresult_max(cv::Mat &image, const std::vector<cv::RotatedRect> detcircles)
{
	cv::RotatedRect temp;
	if (detcircles.size()>0)
	{
		int val = filter_result_max(detcircles,temp);
		// int val =1;
        if(val ==1)
        {
            cv::ellipse(image,temp,cv::Scalar(0,0,255),2);
            temp.angle = 0;
            cv::circle(image,temp.center,2,cv::Scalar(0,255,0),2);
        }
	}

}


void drawresult_common(cv::Mat &image,const std::vector<cv::RotatedRect> detCircles)
{
	cv::RotatedRect temp;
    for (int i = 0; i < detCircles.size(); i++)
    {
        temp.center.x = detCircles[i].center.y;
        temp.center.y = detCircles[i].center.x;
        temp.size.height = detCircles[i].size.width;
        temp.size.width = detCircles[i].size.height;
        temp.angle = -detCircles[i].angle;
        cv::ellipse(image, temp, cv::Scalar(0, 255, 0), 2);
        temp.angle = 0;
        cv::circle(image, temp.center, 2, cv::Scalar(0, 255, 0), 2);
		// std::cout<<temp.size.height<<std::endl;
    }

}

int main(int argc, char** argv) {
	ros::init(argc,argv,"ammed_detect");
	ros::NodeHandle nn;
	image_transport::ImageTransport it(nn);
	image_transport::Subscriber image_left;
	image_transport::Subscriber image_right;
    cv::RotatedRect l_rect;  // 默认构造函数创建一个旋转矩形


    l_rect.center = cv::Point2f(0, 0);
    l_rect.size = cv::Size2f(0, 0);
    l_rect.angle = 0;
        cv::RotatedRect r_rect;  // 默认构造函数创建一个旋转矩形


    l_rect.center = cv::Point2f(0, 0);
    l_rect.size = cv::Size2f(0, 0);
    l_rect.angle = 0;
	image_left = it.subscribe("/airsim_node/drone_1/front_left/Scene",1,image_left_callback);
	// image_right = it.subscribe("/airsim_node/drone_1/front_right/Scene",1,image_right_callback);

    AAMED aamedl(480,640);
    AAMED aamedr(480,640);
    aamedl.SetParameters(CV_PI / 3, 3.4, 0.8); 
    aamedr.SetParameters(CV_PI / 3, 3.4, 0.8); 
	while(ros::ok())
	{	if (!left_img.empty())
		// if(!left_img.empty()&&!right_img.empty())
		{

			cv::Mat blankImage1(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
			cv::Mat blankImage2(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
            pre_lab(left_img,imgL,false);
			aamedl.run_FLED(imgL) ;// run AAMED

			// drawresult_max(blankImage1,aamedl.detEllipses);
			drawresult_common(blankImage1,aamedl.detEllipses);

			cv::imshow("left",blankImage1);

			//开放右边
			// pre_lab(right_img,imgR,false);
			// aamedr.run_FLED(imgR);
			// drawresult(blankImage2,aamedr.detEllipses);
			// cv::imshow("right",blankImage2);
			cv::waitKey(5);
		}
		ros::spinOnce();
	}

	return 0;
}