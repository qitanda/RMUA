
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

void pre_lab(const cv::Mat inimage, cv::Mat &outimage,bool blur)
{

        cv::Mat  hsvImage,yellowTempMat,  blur_img,thrimg, blurMat;

        cv::cvtColor(inimage, hsvImage, cv::COLOR_BGR2HSV);

        // 定义颜色范围
        cv::Scalar lower_yellow(20, 100, 100);  // 黄色的下限值
        cv::Scalar upper_yellow(40, 255, 255);  // 黄色的上限值
        cv::Scalar lower_light_yellow(20, 100, 150);  // 亮黄色的下限值
        cv::Scalar upper_light_yellow(40, 255, 255);  // 亮黄色的上限值
        cv::Scalar lower_pale_yellow(20, 50, 200);  // 淡黄色的下限值
        cv::Scalar upper_pale_yellow(40, 255, 255);  // 淡黄色的上限值

        // 根据颜色范围进行阈值化
        cv::Mat yellowMask, lightYellowMask, paleYellowMask;
        cv::inRange(hsvImage, lower_yellow, upper_yellow, yellowMask);
        cv::inRange(hsvImage, lower_light_yellow, upper_light_yellow, lightYellowMask);
        cv::inRange(hsvImage, lower_pale_yellow, upper_pale_yellow, paleYellowMask);

        // 将所有阈值化后的图像合并
        cv::Mat mergedMask = yellowMask | lightYellowMask | paleYellowMask;

        // 通过与原始图像进行按位与操作，提取出合并后的黄色部分
        cv::bitwise_and(inimage, inimage, yellowTempMat, mergedMask);
        if (blur ==true)
        {
            cv::Mat gray_img;
            cv::threshold(yellowTempMat,thrimg,25,255,CV_THRESH_BINARY);
            cv::medianBlur(thrimg, blurMat,3);
            // cv::bilateralFilter(blurMat,blur_img,5,100,100);


            // cv::medianBlur(yellowTempMat, blurMat,3);
            // cv::bilateralFilter(blurMat,blur_img,5,100,100);
            // cv::GaussianBlur(yellowTempMat,blur_img, cv::Size(3,3), 80,80);

            // cv::bilateralFilter(yellowTempMat,blur_img,5,100,100);
            outimage = blurMat;
            cv::imshow("thrimg",outimage);
            cv::imshow("yellow",yellowTempMat);
            cv::waitKey(5);
        }
        else{
            outimage = yellowTempMat;
            cv::imshow("thrimg",outimage);
            cv::waitKey(5);
        }

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
            outcircle.center.x = incircles[index].center.y;
            outcircle.center.y = incircles[index].center.x;
            outcircle.size.height = incircles[index].size.width;
            outcircle.size.width = incircles[index].size.height;
            outcircle.angle = -incircles[index].angle;
            return 1;
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
    }

}

int main(int argc, char** argv) {
	ros::init(argc,argv,"yellow_detect");
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
            pre_lab(left_img,imgL,true);
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