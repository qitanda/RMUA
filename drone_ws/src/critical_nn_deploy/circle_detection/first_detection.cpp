#include "first_detection.hpp"

FirstDetector::FirstDetector()
{
	aamed->SetParameters(CV_PI / 3, 3.4, 0.8); 
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

void FirstDetector::_pre(const cv::Mat inimage, cv::Mat &outimage,int mode_flag)
{

    if( mode_flag ==1)
        _pre_lab(inimage,outimage);
    else if (mode_flag ==2)
        _pre_yuv(inimage,outimage);
    else if(mode_flag ==3 ||mode_flag ==4 ||mode_flag ==5 )
        _pre_gray(inimage,outimage);
    else 
        printf("error! wrong mode_flag");

}

void FirstDetector::_pre_lab(const cv::Mat inimage, cv::Mat &outimage)
{
	cv::Mat blurimg,blur1_img,blur2_img,Labimg, 
		thrimg,thrimg2,thrimg3,eroimg,diaimg,blur3_img,blur4_img;
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
    cv::bilateralFilter(blur3_img,blur4_img,5,100,100);
    outimage = blur4_img;
}
void FirstDetector::_pre_yuv(const cv::Mat inimage, cv::Mat &outimage)
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

        cv::threshold(yellowTempMat,thrimg,25,255,CV_THRESH_BINARY);
        cv::medianBlur(thrimg, blurMat,3);

        outimage = blurMat;
        // cv::imshow("thrimg",outimage);
        // cv::imshow("yellow",yellowTempMat);
        // cv::waitKey(5);
}
void FirstDetector::_pre_gray(const cv::Mat inimage,cv::Mat &outimage)
{
	cv::Mat hsvImage,thrimg;
    cv::cvtColor(inimage, hsvImage, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsv;
    cv::split(hsvImage,hsv);
    cv::threshold(hsv[2],thrimg,160,255,CV_THRESH_BINARY);
    outimage = thrimg;
}

void FirstDetector::_filter_result_max(std::vector<cv::RotatedRect> incircles,cv::RotatedRect &outcircle)
{
	float max=0;
	int index=-1;
	for ( int i=0;i<incircles.size();i++)
	{
		if((incircles[i].size.height + incircles[i].size.width) > max)
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

void FirstDetector::_filter_result(std::vector<cv::RotatedRect> inCircles, std::vector<cv::RotatedRect> &outCircles, int target_ID)
{
    std::vector<cv::RotatedRect> tempCircles ;
	// std::cout<<"in:"<<inCircles.size()<<std::endl;
    int group[10] = {0};
    for (int i = 0; i < inCircles.size(); i++)
    {
        for (int j = i + 1; j < inCircles.size(); j++)
        {
            float center_dist = sqrt(powf((inCircles[i].center.x - inCircles[j].center.x), 2) + powf((inCircles[i].center.y - inCircles[j].center.y), 2));
            if (center_dist < 25 && group[i] !=-1)
                {
                    group[i] = j;
                    group[j] = -1; //代表已经分了组了
                }
        }
    }
    float max=0;
	int index=-1;
	for ( int i=0;i<inCircles.size();i++)
	{
        if(group[i]==0)
        {
            //首先根据单圈算出圆心坐标是不是target_ID范围内。
            //如果是
                if((3.14159*inCircles[i].size.height*inCircles[i].size.width) > max)
                    {
                        max = 3.14159*inCircles[i].size.height*inCircles[i].size.width;
                        index = i;
                    }
        }
        else if(group[i]>0)
        {
            int k=0;
            //首先看哪个大
            if((inCircles[i].size.height*inCircles[i].size.width) > inCircles[group[i]].size.height*inCircles[group[i]].size.width)
            {
                k=i;
                //首先根据单圈算出圆心坐标是不是target_ID范围内。
                //如果是
                    if((1.5*3.114159*inCircles[k].size.height*inCircles[k].size.width) > max)
                        {
                            max = 1.5*3.114159*inCircles[k].size.height*inCircles[k].size.width;
                            index = k;
                        }
            }
            else 
            {
             	k = group[i];
                    if((1.5*3.114159*inCircles[k].size.height*inCircles[k].size.width) > max)
                        {
                            max = 1.5*3.114159*inCircles[k].size.height*inCircles[k].size.width;
                            index = i;
                        }
            }
        }

	}
	if(index==-1) //没有符合要求的
	{
        tempCircles.clear(); // 这将确保 tempCircles 为空
	}
    else 
    {
        if(group[index] ==0)//没有分组
        {
            tempCircles.clear(); // 这将确保 tempCircles 为空
            tempCircles.push_back(inCircles[index]);
        }
        else 
        {
            if((inCircles[index].size.height+inCircles[index].size.width) < inCircles[group[index]].size.height+inCircles[group[index]].size.width)
            {
                tempCircles.clear(); // 这将确保 tempCircles 为空
                tempCircles.push_back(inCircles[index]);
                tempCircles.push_back(inCircles[group[index]]);
            }
            else
            {
                tempCircles.clear(); // 这将确保 tempCircles 为空
                tempCircles.push_back(inCircles[group[index]]);
                tempCircles.push_back(inCircles[index]);
            }
        }
    }
    outCircles = tempCircles;
	// std::cout<<"out:"<<outCircles.size()<<std::endl;
}

void FirstDetector::run(const cv::Mat inimage, const cv::Mat disparity, int mode_flag)
{
    cv::Mat mid_img;
    _pre(inimage,mid_img,mode_flag);
	// ROS_INFO("1");
	// cv::imshow("mid_img",mid_img);
    if (mode_flag ==1 ||mode_flag ==2)
       {
		 aamed->run_FLED(mid_img);
		_filter_result(aamed->detEllipses,resultCircles,0);
	   }
    else if (mode_flag ==3 ||mode_flag ==4 ||mode_flag ==5)
    {

        cv::Mat disp_8u,disp_8u_resize,eroimg,diaimg,resultimage,invimage;
        disparity.convertTo(disp_8u, CV_8U);
        cv::Mat dilateKernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
        cv::medianBlur(disp_8u*50,disp_8u_resize, 3);
        eroimg = disp_8u_resize & mid_img;
        cv::dilate(eroimg,diaimg , dilateKernel); //执行膨胀操作
        resultimage = diaimg;
        cv::threshold(resultimage,invimage,105,255,CV_THRESH_BINARY);
        cv::medianBlur(invimage,invimage, 3);
        aamed->run_FLED(invimage) ;// run AAMED
		_filter_result(aamed->detEllipses,resultCircles,0);
        // drawresult_max(blankImage1,aamedl.detEllipses);
        // drawresult_common(blankImage1,aamedl.detEllipses);

        // cv::imshow("result1`",blankImage1);
        // cv::imshow("shicha",disp_8u*50);
        // cv::imshow("invimage",invimage);
        // cv::imshow("disp_8u_resize",disp_8u_resize);
        // cv::imshow("diaimg",diaimg);
        // cv::imshow("hsv[2]",hsv[2]);
        // cv::waitKey(1); 
    }
    else
        printf("error! wrong mode_flag\n");
	cv::waitKey(1);
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
    if (detCircles.size() > 2 || detCircles.size() == 0) return {};
    // if (pow(detCircles[0].center.x - detCircles[1].center.y, 2) + pow(detCircles[0].center.y - detCircles[1].center.y, 2) > 100) return;
    cv::RotatedRect target_circle, small_circle;
    // if (detCircles.size() == 2 && detCircles[0].size.area() > detCircles[1].size.area()) 
    // {
    //     target_circle = detCircles[0];
    // }
    // else if (detCircles.size() == 2 && detCircles[0].size.area() < detCircles[1].size.area()) 
    // {
    //     target_circle = detCircles[1];
    // }
    if (detCircles.size() == 1) 
    {
        target_circle = detCircles[0];
        small_circle.center.x = target_circle.center.x;
        small_circle.center.y = target_circle.center.y;
        small_circle.size.width = target_circle.size.width*1/2;
        small_circle.size.height= target_circle.size.height*1/2;
        small_circle.angle= target_circle.angle;
        detCircles.clear();
        detCircles.push_back(small_circle);
        detCircles.push_back(target_circle);
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
    cv::imshow("circle_extract", blank);
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



