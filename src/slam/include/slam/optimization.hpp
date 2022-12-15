#ifndef SLAM_OPTIMIZATION_H_
#define SLAM_OPTIMIZATION_H_

#include "utils.hpp"

#include <iostream>

#include <vector>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <opencv2/opencv.hpp>

Eigen::Matrix4f optimize(
  Eigen::Matrix4f transform_estimation, // Estimation of position + orientation of the source cloud relative to the target cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_edge_points, // Source cloud (in the reference of the sensor)
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_flat_points,
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_edge_points,
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_flat_points,
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_target_edge,
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_target_flat,
  float k
);
void compute_edge_coeffs(
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_edge_points,
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_edge_points,
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_target_edge,
  std::vector<pcl::PointXYZI>* coeffs_corr,
  std::vector<pcl::PointXYZI>* points_corr, 
  std::vector<float>* weights_corr,
  float transform[6],
  float k
);
void compute_flat_coeffs(
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_flat_points,
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_flat_points,
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_target_flat,
  std::vector<pcl::PointXYZI>* coeffs_corr, 
  std::vector<pcl::PointXYZI>* points_corr, 
  std::vector<float>* weights_corr,
  float transform[6],
  float k
);
void transformPoint(pcl::PointXYZI* p_in, pcl::PointXYZI* p_out, float transform[6]);

#endif