#include "utils.hpp"

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