#ifndef SLAM_UTILS_H_
#define SLAM_UTILS_H_

#include <iostream>
#include <memory>
#include <deque>
#include <cstdint>
#include <utility>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <thread>
#include <chrono>
#include <mutex>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "slam/msg/cloud.hpp"
#include "autonomous_interfaces/srv/slam_save_map.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/impl/filter.hpp> // for remove NaN ?
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/pcl_base.h>


struct PointXYZIRT {
	PCL_ADD_POINT4D
	PCL_ADD_INTENSITY
	uint16_t ring;
	float time;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
  (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
  (uint16_t, ring, ring) (float, time, time)
)

class Timer {
public:
  Timer() {
    start_time_point = std::chrono::high_resolution_clock::now();
  }
  ~Timer() {
    Stop();
  }

  void Stop() {

    auto end_time_point = std::chrono::high_resolution_clock::now();

    auto start = std::chrono::time_point_cast<std::chrono::milliseconds>(start_time_point).time_since_epoch().count();
    auto end = std::chrono::time_point_cast<std::chrono::milliseconds>(end_time_point).time_since_epoch().count();

    auto duration = end - start;

    std::cout << duration << " ms\n";
  }
private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_point;
};

Eigen::Matrix4f inverseTransformation(Eigen::Matrix4f m);

Eigen::Matrix4f getDifferenceTransformation(Eigen::Matrix4f m0, Eigen::Matrix4f m1);

Eigen::Vector3f getAnglesFromMatrix(Eigen::Matrix3f rot);

Eigen::Matrix3f getMatrixFromAngles(Eigen::Vector3f angles);

Eigen::Matrix4f getMatrixFromTransform(float transform[6]);

void octreeVoxelGrid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out, float resolution);

#endif