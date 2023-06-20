#include "aut_local_mapping/voxel_grid.h"

#include <iostream>
#include <vector>
#include <array>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "aut_utils/utils.h"

namespace aut_local_mapping {

VoxelGrid::VoxelGrid() {
  initialized = false;
  scale = 0.1f;
  central_pose = Eigen::Matrix4f::Identity();
}

void VoxelGrid::AddObservation(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Matrix4f pose, std::int32_t capture_time_sec, std::uint32_t capture_time_nanosec) {
  if (!initialized) {
    // std::cout << "Initialize...\n";
    Initialize(pose, capture_time_sec, capture_time_nanosec);
  }

  std::uint32_t time_since_init_ms = ComputeTime(capture_time_sec, capture_time_nanosec);
  Eigen::Matrix4f transform_center_current = aut_utils::GetDifferenceTransformation(central_pose, pose);

  if (std::abs(transform_center_current(0, 3)) > 1.0f || 
      std::abs(transform_center_current(1, 3)) > 1.0f || 
      std::abs(transform_center_current(2, 3)) > 1.0f) {
    ChangeCenter(transform_center_current);
    transform_center_current = aut_utils::GetDifferenceTransformation(central_pose, pose);
  }

  // std::cout << "Time since start : " << time_since_init_ms << " ms\n";

  InsertPointCloud(point_cloud, transform_center_current, time_since_init_ms);
}

void VoxelGrid::Initialize(Eigen::Matrix4f pose, std::int32_t capture_time_sec, std::uint32_t capture_time_nanosec) {
  central_pose = pose;
  init_time_sec = capture_time_sec;
  init_time_nanosec = capture_time_nanosec;
  initialized = true;
  // std::cout << "Initialized\n";
}

std::uint32_t VoxelGrid::ComputeTime(std::int32_t capture_time_sec, std::uint32_t capture_time_nanosec) { 
  std::uint32_t result = 0;

  std::int32_t delta_time_sec = capture_time_sec - init_time_sec;
  std::int32_t delta_time_nanosec = (std::int32_t) (capture_time_nanosec - init_time_nanosec);

  if (delta_time_nanosec < 0) {
    delta_time_nanosec += 1000000000;
    delta_time_sec -= 1;
  }

  if (delta_time_sec < 0) {
    return result;
  }
  
  result = delta_time_sec * 1000 + delta_time_nanosec / 1000000;
  return result;
}

void VoxelGrid::ChangeCenter(Eigen::Matrix4f transform_center_current) {

  int delta_idx_x = (int) (transform_center_current(0, 3) / scale);
  int delta_idx_y = (int) (transform_center_current(1, 3) / scale);
  int delta_idx_z = (int) (transform_center_current(2, 3) / scale);

  float delta_x = delta_idx_x * scale;
  float delta_y = delta_idx_y * scale;
  float delta_z = delta_idx_z * scale;

  central_pose(0, 3) += delta_x;
  central_pose(1, 3) += delta_y;
  central_pose(2, 3) += delta_z;

  std::array<std::uint32_t, 1000000> temp_grid = grid;

  for (int z = 0; z < 100; ++z) {
    for (int y = 0; y < 100; ++y) {
      for (int x = 0; x < 100; ++x) {
        int new_x = x + delta_idx_x;
        int new_y = y + delta_idx_y;
        int new_z = z + delta_idx_z;

        if (new_x < 0 || new_x >= 100 ||
            new_y < 0 || new_y >= 100 ||
            new_z < 0 || new_z >= 100) {
          grid[x + 100 * (y + 100 * z)] = 0;
        } else {
          grid[x + 100 * (y + 100 * z)] = temp_grid[new_x + 100 * (new_y + 100 * new_z)];
        }
      }
    }
  }
}

void VoxelGrid::InsertPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Matrix4f transform_center_current, std::uint32_t time_since_init_ms) {

  // std::cout << "Insert point cloud \n";

  Eigen::Affine3f affine_center_current;
  affine_center_current.matrix() = transform_center_current;

  for (int i = 0; i < point_cloud->points.size(); ++i) {
    Eigen::Vector3f point(point_cloud->points[i].x, point_cloud->points[i].y, point_cloud->points[i].z);

    float squared_dist = point.squaredNorm();

    if (squared_dist > 100) {
      continue;
    }

    Eigen::Vector3f new_point = affine_center_current * point;

    Eigen::Vector3i indexes = (new_point / 0.1).cast<int>();
    indexes.array() += 50;

    if (indexes(0) < 0 || indexes(0) >= 100 ||
        indexes(1) < 0 || indexes(1) >= 100 ||
        indexes(2) < 0 || indexes(2) >= 100) {
      // std::cout << "continue :" << indexes << "\n";
      continue;
    }

    grid[indexes(0) + 100 * (indexes(1) + 100 * indexes(2))] = time_since_init_ms;
  }

  Eigen::Affine3f affine_current_center;
  affine_current_center.matrix() = aut_utils::InverseTransformation(transform_center_current);

  for (int z = 0; z < 100; ++z) {
    for (int y = 0; y < 100; ++y) {
      for (int x = 0; x < 100; ++x) {
        std::int32_t voxel_delta_time = (std::int32_t) time_since_init_ms - grid[x + 100 * (y + 100 * z)];
        if (grid[x + 100 * (y + 100 * z)] == 0) {
          continue;
        } else if (voxel_delta_time > 15000) {
          grid[x + 100 * (y + 100 * z)] = 0;
        } else {
          // Eigen::Vector3f point((x - 50) * 0.1 + 0.05, (y - 50) * 0.1 + 0.05, (z - 50) * 0.1 + 0.05);
          // point = affine_current_center * point;
          // float range = point.norm();
          // float angle = std::asin(point(2) / range);
          // if (std::abs(angle * 180 / M_PI) <= 15) {
          //   std::int32_t updated = (std::int32_t) grid[x + 100 * (y + 100 * z)] - voxel_delta_time;
          //   grid[x + 100 * (y + 100 * z)] = updated < 0 ? 0 : updated;
          // }
        } 
      }
    }
  }
}

void VoxelGrid::GetOccupied(std::vector<Eigen::Matrix4f>& occupied_cells) {
  for (int z = 0; z < 100; ++z) {
    for (int y = 0; y < 100; ++y) {
      for (int x = 0; x < 100; ++x) {
        if (grid[x + 100 * (y + 100 * z)] != 0) {
          Eigen::Vector3f point_trans((x - 50) * 0.1 + 0.05, (y - 50) * 0.1 + 0.05, (z - 50) * 0.1 + 0.05);
          Eigen::Matrix4f voxel_pose = Eigen::Matrix4f::Identity();
          voxel_pose.block<3, 1>(0, 3) = point_trans;
          occupied_cells.push_back(central_pose * voxel_pose);
        }
      }
    }
  }
}

}  // namespace aut_local_mapping