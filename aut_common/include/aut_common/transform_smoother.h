#ifndef AUT_COMMON_TRANSFORM_SMOOTHER_H_
#define AUT_COMMON_TRANSFORM_SMOOTHER_H_

#include <memory>
#include <deque>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace aut_common {

class TransformSmoother {

 public:
  explicit TransformSmoother();

  void AddOdometry(const nav_msgs::msg::Odometry::SharedPtr odometry_msg);
  void AddPoseWithCovarianceStamped(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_with_covariance_stamped_msg);
  void AddTransformStamped(const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_msg);

  bool CanSmooth(const rclcpp::Time time);
  Eigen::Affine3f GetSmooth(const rclcpp::Time time);

 private:

  struct TimedAffine {
    Eigen::Affine3f affine;
    rclcpp::Time time;
    TimedAffine();
    TimedAffine(Eigen::Affine3f affine, rclcpp::Time time);
    TimedAffine(const TimedAffine& timed_affine);
    TimedAffine& operator=(TimedAffine timed_affine);
    ~TimedAffine();
  };

  void AddTimedAffine(TimedAffine timed_affine);

  double max_window_time_;
  double tolerance_time_;

  rclcpp::Time oldest_time_;
  rclcpp::Time newest_time_;
  std::deque<TimedAffine> timed_affine_window_;

  // mutex
  std::mutex state_mtx_;
};

}  // namespace aut_common

#endif  // AUT_COMMON_TRANSFORM_SMOOTHER_H_