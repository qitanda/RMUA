#ifndef GET_WAY_POINTS_H
#define GET_WAY_POINTS_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

namespace robot_decision
{
    void get_way_points(const cv::Point3d &P0, const cv::Point3d &P1, const cv::Point3d &P2, const double &yaw, std::vector<cv::Point3d> &way_points)
    {
        cv::Point2d p0 = cv::Point2d(P0.x, P0.y);
        cv::Point2d p1 = cv::Point2d(P0.x + 1 * std::cos(yaw), P0.y + 1 * std::sin(yaw));
        cv::Point2d p2 = cv::Point2d(P1.x, P1.y);
        cv::Point2d p3 = cv::Point2d(P2.x, P2.y);
        Eigen::Vector3d p1_eigen(p0.x, p0.y, 1);
        Eigen::Vector3d p2_eigen(p1.x, p1.y, 1);
        Eigen::Vector3d p3_eigen(p2.x, p2.y, 1);
        Eigen::Vector3d p4_eigen(p3.x, p3.y, 1);
        Eigen::Vector2d l1_vec(p1.x - p0.x, p1.y - p0.y);
        Eigen::Vector2d l2_vec(p3.x - p2.x, p3.y - p2.y);
        Eigen::Vector3d l1 = p1_eigen.cross(p2_eigen);
        Eigen::Vector3d l2 = p3_eigen.cross(p4_eigen);
        double dh = P1.z - P0.z;
        double z = P0.z + dh * 0.5;
        cv::Point3d crossPoint;
        if (l1_vec.dot(l2_vec) <= 0)
        {
            Eigen::Vector3d p_cross = l1.cross(l2);
            cv::Point3d crossPoint_1 = cv::Point3d(p_cross[0] / p_cross[2], p_cross[1] / p_cross[2], z);
            crossPoint = crossPoint_1;
        }
        else
        {
            Eigen::Vector3d l1_(l2[1], -l2[0], l2[0] * p0.y - l2[1] * p0.x);
            Eigen::Vector3d p_cross = l1_.cross(l2);
            cv::Point3d crossPoint_1 = cv::Point3d(p_cross[0] / p_cross[2], p_cross[1] / p_cross[2], z);
            cv::Point3d crossPoint_2 = cv::Point3d(0.5 * crossPoint_1.x + 0.5 * p2.x, 0.5 * crossPoint_1.y + 0.5 * p2.y, z);
            crossPoint = crossPoint_2;
        }
        double dh_1 = crossPoint.z - P0.z;
        double dx_1 = crossPoint.x - P0.x;
        double dy_1 = crossPoint.y - P0.y;
        double dh_2 = crossPoint.z - P1.z;
        double dx_2 = crossPoint.x - P1.x;
        double dy_2 = crossPoint.y - P1.y;
        double dis1 = std::sqrt(std::pow(dx_1, 2) + std::pow(dy_1, 2));
        double dis2 = std::sqrt(std::pow(dx_2, 2) + std::pow(dy_2, 2));
        if (dis1 > dis2)
        {
            cv::Point3d p_1 = cv::Point3d(P0.x + 0.333 * dx_1, P0.y + 0.333 * dy_1, P0.z + 0.333 * dh_1);
            cv::Point3d p_2 = cv::Point3d(P0.x + 0.667 * dx_1, P0.y + 0.667 * dy_1, P0.z + 0.667 * dh_1);
            cv::Point3d p_3 = cv::Point3d(P1.x + 0.5 * dx_2, P1.y + 0.5 * dy_2, P1.z + 0.5 * dh_2);
            way_points.push_back(p_1);
            way_points.push_back(p_2);
            way_points.push_back(p_3);
        }
        else
        {
            cv::Point3d p_1 = cv::Point3d(P0.x + 0.5 * dx_1, P0.y + 0.5 * dy_1, P0.z + 0.5 * dh_1);
            cv::Point3d p_2 = cv::Point3d(P1.x + 0.667 * dx_2, P1.y + 0.667 * dy_2, P1.z + 0.667 * dh_2);
            cv::Point3d p_3 = cv::Point3d(P1.x + 0.333 * dx_2, P1.y + 0.333 * dy_2, P1.z + 0.333 * dh_2);
            way_points.push_back(p_1);
            way_points.push_back(p_2);
            way_points.push_back(p_3);
        }
    }
}

#endif