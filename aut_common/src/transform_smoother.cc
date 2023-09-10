#include "aut_common/transform_smoother.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "aut_utils/utils.h"

namespace aut_common {

TransformSmoother::TimedAffine::TimedAffine() : affine(), time() {}

TransformSmoother::TimedAffine::TimedAffine(Eigen::Affine3f affine, rclcpp::Time time) : affine(affine), time(time) {}

TransformSmoother::TimedAffine::TimedAffine(const TimedAffine& timed_affine) : affine(timed_affine.affine), time(timed_affine.time) {}

TransformSmoother::TimedAffine& TransformSmoother::TimedAffine::operator=(TimedAffine timed_affine) {
  affine = timed_affine.affine;
  time = timed_affine.time;
  return *this;
}

TransformSmoother::TimedAffine::~TimedAffine() {}

TransformSmoother::TransformSmoother() : max_window_time_(5.0), tolerance_time_(0.05), oldest_time_(), newest_time_() {}

void TransformSmoother::AddOdometry(const nav_msgs::msg::Odometry::SharedPtr odometry_msg) {
  Eigen::Affine3f affine = aut_utils::PoseToAffine(odometry_msg->pose.pose);
  rclcpp::Time time(odometry_msg->header.stamp);
  TimedAffine timed_affine(affine, time);
  AddTimedAffine(timed_affine);
}

void TransformSmoother::AddPoseWithCovarianceStamped(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_with_covariance_stamped_msg) {
  Eigen::Affine3f affine = aut_utils::PoseToAffine(pose_with_covariance_stamped_msg->pose.pose);
  rclcpp::Time time(pose_with_covariance_stamped_msg->header.stamp);
  TimedAffine timed_affine(affine, time);
  AddTimedAffine(timed_affine);
}

void TransformSmoother::AddTransformStamped(const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_msg) {
  Eigen::Affine3f affine = aut_utils::TransformToAffine(transform_stamped_msg->transform);
  rclcpp::Time time(transform_stamped_msg->header.stamp);
  TimedAffine timed_affine(affine, time);
  AddTimedAffine(timed_affine);
}

void TransformSmoother::AddTimedAffine(TransformSmoother::TimedAffine timed_affine) {
  std::lock_guard<std::mutex> lock(state_mtx_);
  if (timed_affine_window_.empty()) {
    newest_time_ = timed_affine.time;
    oldest_time_ = timed_affine.time;
    timed_affine_window_.push_back(timed_affine);
    return;
  }

  if (timed_affine.time < newest_time_) {
    newest_time_ = timed_affine.time;
    timed_affine_window_.push_back(timed_affine);

    while ((newest_time_ - oldest_time_).seconds() > max_window_time_) {
      timed_affine_window_.pop_front();
      oldest_time_ = timed_affine_window_.front().time;
    }
  }
}

bool TransformSmoother::CanSmooth(const rclcpp::Time time) {
  std::lock_guard<std::mutex> lock(state_mtx_);
  return time.seconds() < newest_time_.seconds() + tolerance_time_ && time.seconds() > oldest_time_.seconds() - tolerance_time_;
}

Eigen::Affine3f TransformSmoother::GetSmooth(const rclcpp::Time time) {
  std::lock_guard<std::mutex> lock(state_mtx_);
  
  if (!CanSmooth(time)) {
    return Eigen::Affine3f::Identity();
  }

  if (time >= newest_time_) {
    return timed_affine_window_.back().affine;
  }

  if (time <= oldest_time_) {
    return timed_affine_window_.front().affine;
  }

  if (static_cast<int>(timed_affine_window_.size()) == 1) {
    return timed_affine_window_.front().affine;
  }

  auto before_it = timed_affine_window_.begin();
  for (auto it = before_it; it != timed_affine_window_.end(); ++it) {
    if ((*it).time == time) {
      return (*it).affine;
    } else if ((*it).time < time) {
      before_it = it;
    } else {
      break;
    }
  }

  Eigen::Affine3f before_affine = (*before_it).affine;
  rclcpp::Time before_time = (*before_it).time;
  auto after_it = before_it + 1;
  Eigen::Affine3f after_affine = (*after_it).affine;
  rclcpp::Time after_time = (*after_it).time;

  Eigen::Quaternionf before_q(before_affine.linear());
  Eigen::Quaternionf after_q(after_affine.linear());

  Eigen::Vector3f before_t = before_affine.translation();
  Eigen::Vector3f after_t = after_affine.translation();

  double alpha = (time.seconds() - before_time.seconds()) / (after_time.seconds() - before_time.seconds());

  Eigen::Affine3f interp_affine;
  interp_affine.translation() = (1.0 - alpha) * before_t + alpha * after_t;
  interp_affine.linear() = before_q.slerp(alpha, after_q).toRotationMatrix();
  return interp_affine;
} 

}  // namespace aut_common