#include "first_detection.hpp"

FirstDetector::FirstDetector(bool debug)
{
	aamed->SetParameters(CV_PI / 3, 3.4, 0.78); 

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

    /*
    1. 将圈分组，如果有一个圈的外圈和内圈同时识别到，分到一起。如果只识别到一个外圈，那就外圈单独成一组。
    2. 每组选择半径较大的那一个圈用来计算圈心坐标。
    3. 计算圈心坐标是否符合。
        - 根据大圈算出坐标再订阅vio里程计信息，每个遍历计算与下一个目标点的空间距离误差在2根号3设为3.5的留下来。
        - 此时保留最大的圈对应的那一组。（因为最大的圈在飞机前方也就是我们下一个目标圈）。
        - 如果最终没有符合要求，直接设为0.
    4. 最后输出的ourcircles有三种情况：
        - size =2，代表一个圈的外圈和内圈其中[0]代表外圈
        - size =1，代表一个圈
        - size =0 代表没有符合的。
    */
void FirstDetector::_filter_result(std::vector<cv::RotatedRect> inCircles, std::vector<cv::RotatedRect> &outCircles, int target_ID)
{
    std::vector<cv::RotatedRect> tempCircles ;
	std::cout<<"in:"<<inCircles.size()<<std::endl;
    int group[10] = {0};
    for (int i = 0; i < inCircles.size(); i++)
    {
        for (int j = i + 1; j < inCircles.size(); j++)
        {
            float center_dist = sqrt(powf((inCircles[i].center.x - inCircles[j].center.x), 2) + powf((inCircles[i].center.y - inCircles[j].center.y), 2));
            if (center_dist < 5 && group[i] !=-1)
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
                if((inCircles[i].size.height+inCircles[i].size.width) > max)
                    {
                        max = inCircles[i].size.height+inCircles[i].size.width;
                        index = i;
                    }
        }
        else if(group[i]>0)
        {
            int k=0;
            //首先看哪个大
            if((inCircles[i].size.height+inCircles[i].size.width) > inCircles[group[i]].size.height+inCircles[group[i]].size.width)
            {
                k=i;
                //首先根据单圈算出圆心坐标是不是target_ID范围内。
                //如果是
                    if((inCircles[k].size.height+inCircles[k].size.width) > max)
                        {
                            max = inCircles[k].size.height+inCircles[k].size.width;
                            index = i;
                        }
            }
            else 
            {
                k=group[i];
                if((inCircles[k].size.height+inCircles[k].size.width) > max)
                {
                    max = inCircles[k].size.height+inCircles[k].size.width;
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
            if((inCircles[index].size.height+inCircles[index].size.width) > inCircles[group[index]].size.height+inCircles[group[index]].size.width)
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
	std::cout<<"out:"<<outCircles.size()<<std::endl;
}





void FirstDetector::run(const cv::Mat inimage, const cv::Mat disparity, int mode_flag)
{
    cv::Mat mid_img;
    _pre(inimage,mid_img,mode_flag);
	ROS_INFO("1");
	cv::imshow("mid_img",mid_img);
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
    }
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
FirstDetector::~FirstDetector() {};


