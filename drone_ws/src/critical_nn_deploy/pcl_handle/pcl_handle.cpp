#include "pcl_handle.hpp"

namespace pcl_handle {
    depth_extract::depth_extract() {
        size = cv::Size(640, 480);
    }

    depth_extract::depth_extract(pcl::PointCloud<pcl::PointXYZ> cloud) {
        origin_cloud = cloud;
        size = cv::Size(640, 480);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr depth_extract::outlier_filter(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCDReader reader;
        // reader.read<pcl::PointXYZ> ("../test_pcd.pcd", *cloud_ptr);
        // pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZ>> vox;  //创建滤波对象
        // vox.setInputCloud(*cloud_ptr);            //设置需要过滤的点云给滤波对象
        // vox.setLeafSize(0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
        // vox.filter(*cloud_ptr) ;           //执行滤波处理，存储输出
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
        ror.setInputCloud(cloud_ptr);
        ror.setRadiusSearch(0.1); // 设置半径搜索范围
        ror.setMinNeighborsInRadius(5); // 设置邻域内最小点数阈值
        ror.filter(*filter_cloud_ptr);
        if (filter_cloud_ptr->size() == 0) output_cloud_ptr = filter_cloud_ptr;
        else
        {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(filter_cloud_ptr);
            sor.setMeanK(10);//50
            sor.setStddevMulThresh(0.3);//1.0
            sor.filter(*output_cloud_ptr);
        }
        // std::cout << "PointCloud after filter has: " << output_cloud_ptr->points.size () << " data points." << std::endl; 
        // euler_cluster(output_cloud_ptr);
        // hist = cv::Mat::zeros(size, CV_8UC3);
        // for ()
        // pcl::io::savePCDFileASCII ("../filter_pcd.pcd", *output_cloud_ptr);
        return output_cloud_ptr;
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr depth_extract::euler_cluster(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> vg; // VoxelGrid类在输入点云数据上创建3D体素网格（将体素网格视为一组空间中的微小3D框
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        // vg.setInputCloud (cloud); //输入
        // vg.setLeafSize (0.01f, 0.01f, 0.01f);  // setLeafSize (float lx, float ly, float lz)
        // vg.filter (*cloud_filtered); //输出
        *cloud_filtered = *cloud;
        std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*滤波后
        //创建平面模型分割的对象并设置参数
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::PCDWriter writer;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);    //分割模型
        seg.setMethodType (pcl::SAC_RANSAC);       //随机参数估计方法
        seg.setMaxIterations (100);                //最大的迭代的次数
        seg.setDistanceThreshold (0.02);           //设置阀值
        int i=0, nr_points = (int) cloud_filtered->points.size ();
        while (cloud_filtered->points.size () > 0.3 * nr_points) // 滤波停止条件
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud_filtered); // 输入
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers);
            extract.setNegative (false);
            // Get the points associated with the planar surface
            extract.filter (*cloud_plane);// [平面
            std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
            //  // 移去平面局内点，提取剩余点云
            extract.setNegative (true);
            extract.filter (*cloud_f);
            *cloud_filtered = *cloud_f;
        }
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_filtered);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;   //欧式聚类对象
        ec.setClusterTolerance (0.02);                     // 设置近邻搜索的搜索半径为2cm
        ec.setMinClusterSize (100);                 //设置一个聚类需要的最少的点数目为100
        ec.setMaxClusterSize (25000);               //设置一个聚类需要的最大点数目为25000
        ec.setSearchMethod (tree);                    //设置点云的搜索机制
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中
        //迭代访问点云索引cluster_indices,直到分割出所有聚类
        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
            std::stringstream ss;
            ss << "../cloud_cluster_" << j << ".pcd";
            // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); // 保存文件
            j++;
        }
    }

    void depth_extract::normal_estimate(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
        float radius = 0.3;                   // 设置查找半径范围
        int max_nn = 100;                      // 设置在半径范围内要查询的最大点个数
        std::vector<int> pointIdxRN;           // 保存每个近邻点索引的容器
        std::vector<float> pointRNSqDistance;  // 保存每个近邻点与查找点之间的欧式距离平方的容器
        pcl::PointXYZ searchPoint = cloud->points[int(cloud->points.size() / 2)]; // 需要计算法向量的点
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
        kdtree->setInputCloud(cloud);          // 设置KdTree的输入点云
        Eigen::Vector4f planeParams;           // 拟合平面的单位法向量
        float curvature;                       // 表面曲率
        //----------------------------进行单个点的法向量求解---------------------------
        pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal>n;
        // -------------------------------kdtree混合搜索-------------------------------
        if (kdtree->radiusSearch(searchPoint, radius, pointIdxRN, pointRNSqDistance, max_nn) > 0)
        {
            n.computePointNormal(*cloud, pointIdxRN, planeParams, curvature);
            std::cout << "平面拟合参数为：\n" << planeParams << std::endl;
            std::cout << "平面拟合参数a为：\n" << planeParams[0] << std::endl;
        }
        else
        {
            PCL_ERROR("邻域点数太少，无法进行法向量计算！！！");
            return;
        }
        // pcl::NormalEstimation<:pointxyz pcl::normal> ne;
        // ne.setInputCloud (cloud_xyz);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr depth_extract::PCAPlanFitting(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, double &yaw)
    {
        // pcl::PointCloud<pcl::PointXYZ>::Ptr stereoCloudptr = pcl_Stereocloud.makeShared();
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        //inliers表示误差能容忍的点 记录的是点云的序号
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        //创建分割器
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);//0.01
        seg.setMaxIterations(500);
        seg.setAxis(Eigen::Vector3f(0, 0, 1));
        seg.setEpsAngle(45.0f * (M_PI / 180.0f));
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        // std::cerr << "Model coefficients: " << coefficients->values[0] << " "
        //     << coefficients->values[1] << " "
        //     << coefficients->values[2] << " "
        //     << coefficients->values[3] << std::endl;
        // std::cerr << "ax" << seg.getAxis() << std::endl;
        double angle = acos(coefficients->values[2]);
        if (coefficients->values[0] < 0) angle = -angle;
        if (angle > M_PI/2) angle -= M_PI;
        if (angle < -M_PI/2) angle += M_PI;
        yaw += angle;
        if (yaw < -M_PI) yaw = yaw + M_PI * 2;
        if (yaw > M_PI) yaw = yaw - M_PI * 2;
        std::cerr << "yaw:" << yaw << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);
        //只取inliners中索引对应的点拷贝到inlierPoints中
        pcl::copyPointCloud(*cloud, *inliers, *inlierPoints);
        return inlierPoints;
    }

    void center_extract(const cv::Mat src) {

    }

    depth_extract::~depth_extract() {}
}