#include "first_detection.hpp"

FirstDetector::FirstDetector()
{
	aamed->SetParameters(CV_PI / 3, 3.4, 0.81); 
}

// FirstDetector::FirstDetector(ros::NodeHandle *nh,bool debug1) : first_nh(*nh)  ,debug(debug1)
// {
// 	aamed->SetParameters(CV_PI / 3, 3.4, 0.81); 
// }

void FirstDetector::drawresult_max(cv::Mat &image, const std::vector<cv::RotatedRect> detcircles)
{
	cv::RotatedRect temp;
	if (detcircles.size()>0)
	{
		_filter_result_max(detcircles,temp);
		cv::ellipse(image,temp,cv::Scalar(0,0,255),2);
		temp.angle = 0;
		cv::circle(image,temp.center,2,cv::Scalar(0,255,0),2);
	}
}

void FirstDetector::_pre_lab(const cv::Mat inimage, cv::Mat &outimage,bool detect_blue)
{
	cv::Mat blurimg,blur1_img,blur2_img,Labimg, 
		thrimg,thrimg2,thrimg3,eroimg,diaimg,blur3_img,blur4_img, 
		blue_L, blue_b,blue,blue_erode,blue_dilate,blue_blur;
	//以下是图像预处理
	// cv::medianBlur(inimage,blur1_img,3);
	// cv::medianBlur(blur1_img,blur2_img,3);
	// cv::cvtColor(blur2_img,Labimg,cv::COLOR_BGR2Lab);
	cv::cvtColor(inimage,Labimg,cv::COLOR_BGR2Lab);
	std::vector<cv::Mat> lab;
	cv::split(Labimg,lab);
	cv::threshold(lab[1],thrimg,151,255,CV_THRESH_BINARY);
	cv::threshold(lab[2],thrimg2,180,255,CV_THRESH_BINARY_INV);
	cv::threshold(lab[2],thrimg3,70,255,CV_THRESH_BINARY);
	thrimg=thrimg&(thrimg2&thrimg3);
    cv::Mat erodeKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::erode(thrimg,eroimg,erodeKernel);
	cv::Mat dilateKernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(5, 5));
	cv::dilate(eroimg,diaimg,dilateKernel);
	cv::medianBlur(diaimg,blur3_img, 3);
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
	// cv::imshow("erode",eroimg);
	// cv::imshow("dialage",diaimg);
	// cv::imshow("pre",outimage);
	// cv::waitKey(5);

}

void FirstDetector::_filter_result_max(std::vector<cv::RotatedRect> incircles,cv::RotatedRect &outcircle)
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

	}
}


void FirstDetector::run(const cv::Mat inimage)
{
	cv::Mat mid_img;
	_pre_lab(inimage,mid_img,false);
	aamed->run_FLED(mid_img);
}


void FirstDetector::drawresult_common(cv::Mat &image,const std::vector<cv::RotatedRect> detCircles)
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

void FirstDetector::_ellipse_parameters(const std::vector<cv::RotatedRect> detCircles)
{
    double cx=detCircles[0].center.x,cy=detCircles[0].center.y,a=detCircles[0].size.width/2,b=detCircles[0].size.height/2,rot_ang=detCircles[0].angle*CV_PI/180.0;
    A1=(a*sin(rot_ang))*(a*sin(rot_ang))+(b*cos(rot_ang))*((b*cos(rot_ang)));
    B1=-2*(a*a-b*b)*sin(rot_ang)*cos(rot_ang);
    C1=(a*cos(rot_ang))*(a*cos(rot_ang))+(b*sin(rot_ang))*(b*sin(rot_ang));
    D1=-(2*A1*cx+B1*cy);
    E1=-(2*C1*cy+B1*cx);
    F1=A1*cx*cx+B1*cx*cy+C1*cy*cy-(a*b)*(a*b);
    cx=detCircles[1].center.x;cy=detCircles[1].center.y;a=detCircles[1].size.width/2;b=detCircles[1].size.height/2;rot_ang=detCircles[1].angle*CV_PI/180.0;
    A2=(a*sin(rot_ang))*(a*sin(rot_ang))+(b*cos(rot_ang))*((b*cos(rot_ang)));
    B2=-2*(a*a-b*b)*sin(rot_ang)*cos(rot_ang);
    C2=(a*cos(rot_ang))*(a*cos(rot_ang))+(b*sin(rot_ang))*(b*sin(rot_ang));
    D2=-(2*A2*cx+B2*cy);
    E2=-(2*C2*cy+B2*cx);
    F2=A2*cx*cx+B2*cx*cy+C2*cy*cy-(a*b)*(a*b);
}

std::vector<cv::Point2f> FirstDetector::ellipse_extract(const cv::Mat disparity, std::vector<cv::RotatedRect> detCircles)
{
    // std::cout<<detCircles.size()<<std::endl;
    if (detCircles.size() != 2) return {};
    // if (pow(detCircles[0].center.x - detCircles[1].center.y, 2) + pow(detCircles[0].center.y - detCircles[1].center.y, 2) > 100) return;
    if (detCircles[0].size.area() > detCircles[1].size.area()) 
    {
        cv::RotatedRect detCircles_temp = detCircles[1];
        detCircles[1] = detCircles[0];
        detCircles[0] = detCircles_temp;
    }
    cv::Mat blank(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
    FirstDetector::_ellipse_parameters(detCircles);
    std::vector<cv::Point2f> point_set;
    double fx = 320.0,fy=320.0,cx=320.0,cy=240.0;
    double b = 0.095;
    double baseline = 95;
    for (int i = 0; i < blank.rows; ++i)
        for (int j = 0; j < blank.cols; ++j)
        {
            if (A2*i*i + B2*i*j + C2*j*j + D2*i + E2*j < -F2 && A1*i*i + B1*i*j + C1*j*j + D1*i + E1*j > -F1)
            {
                point_set.push_back(cv::Point2f(j, i));
                cv::circle(blank, cv::Point2f(j, i), 1, cv::Scalar(0, 255, 0), 1);
                double x = (i - cx) / fx;
                double y = (j - cy) / fy;
                double depth = fx * baseline / (disparity.at<float>(i, j));
                // cloud.points[i * disparity.cols + j].x = x * depth;
                // cloud.points[i * disparity.cols + j].y = y * depth;
                // cloud.points[i * disparity.cols + j].z = depth;
                // cloud.points.emplace_back(pcl::PointXYZ(x * depth, y * depth, depth));
            }
        }
    // cv::imshow("circle_extract", blank);
    return point_set;
}

std::vector<cv::Point2f> FirstDetector::single_ellipse_extract(const cv::Mat disparity, std::vector<cv::RotatedRect> detCircles)
{
    // std::cout<<detCircles.size()<<std::endl;
    if (detCircles.size() > 2) return {};
    // if (pow(detCircles[0].center.x - detCircles[1].center.y, 2) + pow(detCircles[0].center.y - detCircles[1].center.y, 2) > 100) return;
    cv::RotatedRect target_circle, small_circle;
    if (detCircles.size() == 2 && detCircles[0].size.area() > detCircles[1].size.area()) 
    {
        target_circle = detCircles[0];
    }
    else if (detCircles.size() == 2 && detCircles[0].size.area() < detCircles[1].size.area()) 
    {
        target_circle = detCircles[1];
    }
    else if (detCircles.size() == 1) target_circle = detCircles[0];
    small_circle.center.x = target_circle.center.x;
    small_circle.center.y = target_circle.center.y;
    small_circle.size.width = target_circle.size.width*1/2;
    small_circle.size.height= target_circle.size.height*1/2;
    small_circle.angle= target_circle.angle;
    detCircles.clear();
    detCircles.push_back(small_circle);
    detCircles.push_back(target_circle);
    cv::Mat blank(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
    FirstDetector::_ellipse_parameters(detCircles);
    std::vector<cv::Point2f> point_set;
    double fx = 320.0,fy=320.0,cx=320.0,cy=240.0;
    double b = 0.095;
    double baseline = 95;
    for (int i = 0; i < blank.rows; ++i)
        for (int j = 0; j < blank.cols; ++j)
        {
            if (A2*i*i + B2*i*j + C2*j*j + D2*i + E2*j < -F2 && A1*i*i + B1*i*j + C1*j*j + D1*i + E1*j > -F1)
            {
                point_set.push_back(cv::Point2f(j, i));
                cv::circle(blank, cv::Point2f(j, i), 1, cv::Scalar(0, 255, 0), 1);
                double x = (i - cx) / fx;
                double y = (j - cy) / fy;
                double depth = fx * baseline / (disparity.at<float>(i, j));
                // cloud.points[i * disparity.cols + j].x = x * depth;
                // cloud.points[i * disparity.cols + j].y = y * depth;
                // cloud.points[i * disparity.cols + j].z = depth;
                // cloud.points.emplace_back(pcl::PointXYZ(x * depth, y * depth, depth));
            }
        }
    // cv::imshow("circle_extract", blank);
    return point_set;
}

cv::Point2f FirstDetector::single_circle_center(std::vector<cv::RotatedRect> detCircles)
{
    if (detCircles.size() > 2) return {};
    // if (pow(detCircles[0].center.x - detCircles[1].center.y, 2) + pow(detCircles[0].center.y - detCircles[1].center.y, 2) > 100) return;
    cv::RotatedRect target_circle, small_circle;
    if (detCircles.size() == 2 && detCircles[0].size.area() > detCircles[1].size.area()) 
    {
        target_circle = detCircles[0];
    }
    else if (detCircles.size() == 2 && detCircles[0].size.area() < detCircles[1].size.area()) 
    {
        target_circle = detCircles[1];
    }
    else if (detCircles.size() == 1) target_circle = detCircles[0];
    return target_circle.center;
}
FirstDetector::~FirstDetector() {};

// int main(int argc, char** argv) {
// 	ros::init(argc,argv,"ammed_detect");
// 	ros::NodeHandle nn;
// 	FirstDetector Left = FirstDetector(&nn,false);
// 	FirstDetector Right = FirstDetector(&nn,false);
// 	image_transport::ImageTransport it(nn);
// 	image_transport::Subscriber image_left;
// 	image_transport::Subscriber image_right;
// 	it.subscribe("/airsim_node/drone_1/front_left/Scene",1,image_left_callback);
// 	it.subscribe("/airsim_node/drone_1/front_right/Scene",1,image_right_callback);

// 	while(ros::ok())
// 	{	
// 		if(!left_img.empty() && !right_img.empty())
// 		{

// 			cv::Mat blankImage1(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
// 			cv::Mat blankImage2(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));

// 			Left.run(left_img) ;// run AAMED
// 			Left.drawresult(blankImage1,Left.aamed->detEllipses);
// 			Right.run(right_img);
// 			Right.drawresult(blankImage2,Right.aamed->detEllipses);


// 			cv::imshow("left",blankImage1);
// 			cv::imshow("right",blankImage2);
// 			cv::waitKey(5);
// 		}
// 		ros::spinOnce();
// 	}

// 	return 0;
// }

