#include "first_detection.hpp"

cv::Mat left_img,right_img;
cv::Mat imgL,imgR;

void image_left_callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImageConstPtr cv_image;
	cv_image = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	left_img = cv_image->image;

	cv::imshow("left_image",left_img);
	cv::waitKey(1);
	
}

void image_right_callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImageConstPtr cv_image;
	cv_image = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	right_img = cv_image->image;
	// cv::imshow("right_image",right_img);
	// cv::waitKey(10);
}


int main(int argc, char** argv) {
	ros::init(argc,argv,"red_detect");
	ros::NodeHandle nn;
	FirstDetector Left = FirstDetector(false);
	image_transport::ImageTransport it(nn);
	image_transport::Subscriber image_left;
	image_left = it.subscribe("/airsim_node/drone_1/front_left/Scene",1,image_left_callback);


    //开放右边
    // FirstDetector Right = FirstDetector(&nn,false);
    // image_transport::Subscriber image_right;
    // it.subscribe("/airsim_node/drone_1/front_right/Scene",1,image_right_callback);
	while(ros::ok())
	{	
		// if(!left_img.empty() && !right_img.empty())
        if(!left_img.empty())
		{

			cv::Mat blankImage1(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
			cv::Mat blankImage2(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));

			Left.run(left_img, blankImage2,2) ;// run AAMED
            Left.drawresult_common(blankImage1,Left.resultCircles);
            cv::imshow("left",blankImage1);
			// Right.run(right_img);
			// Right.drawresult(blankImage2,Right.aamed->detEllipses);
			// cv::imshow("right",blankImage2);
			cv::waitKey(1);
		}
		ros::spinOnce();
	}

	return 0;
}