#ifndef PCL_HANDLE_H
#define PCL_HANDLE_H

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv2/opencv.hpp>


namespace pcl_handle {
    class depth_extract {
    public:
        depth_extract();

        depth_extract(pcl::PointCloud<pcl::PointXYZ> origin_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_filter(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

        pcl::PointCloud<pcl::PointXYZ>::ConstPtr euler_cluster(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

        void center_extract(const cv::Mat src);

        void normal_estimate(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr PCAPlanFitting(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, double &yaw);

        ~depth_extract();

    private:
        cv::Mat hist;
        pcl::PointCloud<pcl::PointXYZ> origin_cloud, output_cloud, temp_cloud;
        cv::Size size;
    };
}

#endif