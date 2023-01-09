#include "utils.hpp"

Eigen::Matrix4f transformStampedToMatrix(geometry_msgs::msg::TransformStamped t) {
  Eigen::Matrix4f matrix;
  Eigen::Quaternionf rot(t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z);
  Eigen::Vector3f trans(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
  Eigen::Affine3f affine;
  affine.translation() = trans;
  affine.linear() = rot.toRotationMatrix();
  matrix = affine.matrix();
  return matrix;
}

Eigen::Matrix4f poseToMatrix(geometry_msgs::msg::Pose pose) {
  Eigen::Matrix4f matrix;
  Eigen::Quaternionf rot(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Vector3f trans(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Affine3f affine;
  affine.translation() = trans;
  affine.linear() = rot.toRotationMatrix();
  matrix = affine.matrix();
  return matrix;
}

Eigen::Matrix4f valuesToMatrix(float x, float y, float z, float rot_x, float rot_y, float rot_z, float rot_w) {
  Eigen::Matrix4f matrix;
  Eigen::Quaternionf rot(rot_w, rot_x, rot_y, rot_z);
  Eigen::Vector3f trans(x, y, z);
  Eigen::Affine3f affine;
  affine.translation() = trans;
  affine.linear() = rot.toRotationMatrix();
  matrix = affine.matrix();
  return matrix;
}

Eigen::Matrix4f inverseTransformation(Eigen::Matrix4f m) {
  Eigen::Matrix4f inv = Eigen::Matrix4f::Identity();
  inv.block<3, 3>(0, 0) = m.block<3, 3>(0, 0).transpose();
  inv.block<3, 1>(0, 3) = - m.block<3, 3>(0, 0).transpose() * m.block<3, 1>(0, 3);
  return inv;
}

Eigen::Matrix4f getDifferenceTransformation(Eigen::Matrix4f m0, Eigen::Matrix4f m1) {
  return inverseTransformation(m0) * m1;
}

Eigen::Vector3f getAnglesFromMatrix(Eigen::Matrix3f rot) {

  float alpha = std::atan2(rot(1, 0), rot(0, 0));
  float beta = std::atan2(-rot(2, 0), std::sqrt(1 - rot(2, 0) * rot(2, 0)));
  float gamma = std::atan2(rot(2, 1), rot(2, 2));

  Eigen::Vector3f angles;
  angles << alpha, beta, gamma;

  return angles;
}

Eigen::Matrix3f getMatrixFromAngles(Eigen::Vector3f angles) {

  Eigen::Matrix3f rot;

  float c1 = std::cos(angles(0));
  float c2 = std::cos(angles(1));
  float c3 = std::cos(angles(2));

  float s1 = std::sin(angles(0));
  float s2 = std::sin(angles(1));
  float s3 = std::sin(angles(2));

  rot(0, 0) = c1 * c2;
  rot(0, 1) = c1 * s2 * s3 - c3 * s1;
  rot(0, 2) = s1 * s3 + c1 * c3 * s2;

  rot(1, 0) = c2 * s1;
  rot(1, 1) = c1 * c3 + s1 * s2 * s3;
  rot(1, 2) = c3 * s1 * s2 - c1 * s3;

  rot(2, 0) = -s2;
  rot(2, 1) = c2 * s3;
  rot(2, 2) = c2 * c3;

  return rot;
}

Eigen::Matrix4f getMatrixFromTransform(float transform[6]) {

  Eigen::Matrix4f m;
  m = Eigen::Matrix4f::Identity();

  Eigen::Vector3f angles(transform[0], transform[1], transform[2]);

  m.block<3, 3>(0, 0) = getMatrixFromAngles(angles);
  m(0, 3) = transform[3];
  m(1, 3) = transform[4];
  m(2, 3) = transform[5];

  return m;
}

void octreeVoxelGrid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out, float resolution) {
  
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_leaf;
  voxel_grid_leaf.setLeafSize(resolution, resolution, resolution);

  float octree_resolution = 8.0f;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree(octree_resolution);
  octree.setInputCloud(cloud_in);
  octree.addPointsFromInputCloud();

  for (auto it = octree.leaf_breadth_begin(); it != octree.leaf_breadth_end(); ++it) {

    pcl::IndicesPtr indexVector(new std::vector<int>);
    pcl::octree::OctreeContainerPointIndices& container = it.getLeafContainer();
    container.getPointIndices(*indexVector);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_leaf(new pcl::PointCloud<pcl::PointXYZI>);
    voxel_grid_leaf.setInputCloud(cloud_in);
    voxel_grid_leaf.setIndices(indexVector);
    voxel_grid_leaf.filter(*filtered_leaf);
    *cloud_out += *filtered_leaf;
  }

}