#ifndef AUT_LOCAL_MAPPING_VOXEL_GRID_H_
#define AUT_LOCAL_MAPPING_VOXEL_GRID_H_

#include <array>
#include <vector>
#include <cstdint>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

namespace aut_local_mapping {

class VoxelGrid {

 public:
  explicit VoxelGrid();  // TODO: See if we need to put "= default" or "= delete"

  void AddObservation(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Matrix4f pose, std::int32_t capture_time_sec, std::uint32_t capture_time_nanosec);

  void Initialize(Eigen::Matrix4f pose, std::int32_t capture_time_sec, std::uint32_t capture_time_nanosec);

  std::uint32_t ComputeTime(std::int32_t capture_time_sec, std::uint32_t capture_time_nanosec);

  void ChangeCenter(Eigen::Matrix4f transform_center_current);

  void InsertPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Matrix4f transform_center_current, std::uint32_t time_since_init_ms);

  void GetOccupied(std::vector<Eigen::Matrix4f>& occupied_cells);

 private:

  bool initialized;

  float scale;

  Eigen::Matrix4f central_pose;

  std::int32_t init_time_sec;
  std::uint32_t init_time_nanosec;

  std::array<std::uint32_t, 1000000> grid;
};

}  // namespace aut_local_mapping

#endif  // AUT_LOCAL_MAPPING_VOXEL_GRID_H_