 #include "aut_local_mapping/local_mapper.h"

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>

#include "aut_utils/utils.h"
#include "aut_local_mapping/voxel_grid.h"
#include "aut_msgs/msg/point_cloud_with_pose.hpp"

namespace aut_local_mapping {

LocalMapper::LocalMapper(const rclcpp::NodeOptions& options)
    : Node("local_mapper", options) {

  n = 0;
  
  callback_group_point_cloud_with_pose_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive
  );

  rclcpp::SubscriptionOptions point_cloud_with_pose_options = rclcpp::SubscriptionOptions();
  point_cloud_with_pose_options.callback_group = callback_group_point_cloud_with_pose_;

  point_cloud_with_pose_subscription_ = this->create_subscription<aut_msgs::msg::PointCloudWithPose>(
    "aut_lidar_odometry/point_cloud_with_pose", 10, 
    std::bind(&LocalMapper::PointCloudWithPoseCallBack, this, std::placeholders::_1),
    point_cloud_with_pose_options
  );

  marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("aut_local_mapping/voxels", 10);
}

void LocalMapper::PointCloudWithPoseCallBack(const aut_msgs::msg::PointCloudWithPose::SharedPtr point_cloud_with_pose_msg) {
  RCLCPP_INFO(this->get_logger(), "Received point cloud");

  std::vector<Eigen::Matrix4f> occupied;
  {

    Timer timer;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_point_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(point_cloud_with_pose_msg->point_cloud, *input_point_cloud);

    Eigen::Matrix4f pose = aut_utils::TransformToMatrix(point_cloud_with_pose_msg->pose);

    grid.AddObservation(input_point_cloud, pose, point_cloud_with_pose_msg->header.stamp.sec, point_cloud_with_pose_msg->header.stamp.nanosec);

    grid.GetOccupied(occupied);
  }

  PublishMarkerArray(occupied);

  RCLCPP_INFO(this->get_logger(), "Size occupied : %d", occupied.size());
}

void LocalMapper::PublishMarkerArray(std::vector<Eigen::Matrix4f>& occupied) {

  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker marker0;
  marker0.header.frame_id = "odom";
  marker0.header.stamp = this->get_clock()->now();
  marker0.ns = std::string("voxel");
  marker0.id = 0;
  marker0.action = 3;
  marker_array.markers.push_back(marker0);

  // visualization_msgs::msg::Marker marker1;

  // marker1.header.frame_id = "odom";
  // marker1.header.stamp = this->get_clock()->now();

  // marker1.ns = std::string("voxel");
  // marker1.id = 0;

  // marker1.type = 1;

  // marker1.action = 0;
  // marker1.pose.position.x = n / 10.0f;
  // ++n;

  // marker1.scale.x = 1.0;
  // marker1.scale.y = 1.0;
  // marker1.scale.z = 1.0;

  // marker1.lifetime.sec = 1;

  // marker1.color.a = 1.0;
  // marker1.color.r = 0.0;
  // marker1.color.g = 1.0;
  // marker1.color.b = 1.0;

  // marker_array.markers.push_back(marker1);

  // marker_array_publisher_->publish(marker_array);

  for (int i = 0; i < occupied.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->get_clock()->now();

    marker.ns = std::string("voxel");
    marker.id = i + 1;
    marker.type = 1;
    marker.action = 0;
    
    marker.pose = aut_utils::MatrixToPose(occupied[i]);

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.lifetime.sec = 1;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    marker_array.markers.push_back(marker);
  }

  marker_array_publisher_->publish(marker_array);

}

}  // namespace aut_local_mapping