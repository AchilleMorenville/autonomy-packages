#include "utils.hpp"

#include <dynamicEDT3D/dynamicEDT3D.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include <octomap/OcTree.h>

using namespace std::chrono_literals;

class MonteCarloLocalization : public rclcpp::Node {

public:

  MonteCarloLocalization() : Node("monte_carlo_localization") {

    n = 0;

    initialized = false;
    map_loaded = false;
    initial_estimation = false;

    sigma_hit = 0.05;
    z_rand = 0.1;
    z_hit = 0.9;
    max_range = 30;

    this->declare_parameter("n_particles", 1000);
    n_particles = this->get_parameter("n_particles").get_parameter_value().get<int>();

    this->declare_parameter("leaf_size", 0.2f);
    leaf_size = this->get_parameter("leaf_size").get_parameter_value().get<float>();

    callback_group_cloud = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions cloud_options = rclcpp::SubscriptionOptions();
    cloud_options.callback_group = callback_group_cloud;

    subscription_cloud_ = this->create_subscription<slam::msg::Cloud>(
      "slam/lidar_odometry", 10, std::bind(&MonteCarloLocalization::cloudHandler, this, std::placeholders::_1), cloud_options
    );

    callback_group_fiducials = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions fiducials_options = rclcpp::SubscriptionOptions();
    fiducials_options.callback_group = callback_group_fiducials;

    subscription_fiducials_ = this->create_subscription<autonomous_interfaces::msg::Fiducials>(
      "spot_driver/fiducials", 10, std::bind(&MonteCarloLocalization::fiducialsHandler, this, std::placeholders::_1), fiducials_options
    );

    allocateMemory();

    loadMap();

  }

private:

  int n;

  bool initial_estimation;
  bool initialized;
  bool map_loaded;

  float sigma_hit;
  float z_rand;
  float z_hit;
  float max_range;

  // Parameters

  int n_particles;

  float leaf_size;

  // Input
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_all_points;
  Eigen::Matrix4f input_robot_odometry;

  // Last
  Eigen::Matrix4f last_robot_odometry;

  // Initial estimation
  Eigen::Matrix4f map_tform_body_init;

  // VoxelGrid
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;

  // Octree & DynamicEDT3D
  std::shared_ptr<octomap::OcTree> map_octree;
  std::shared_ptr<DynamicEDTOctomap> map_edt;

  // Fiducials
  std::unordered_set<int> tag_ids;
  std::unordered_map<int, Eigen::Matrix4f> map_tform_fiducials;

  // Subscription
  rclcpp::Subscription<slam::msg::Cloud>::SharedPtr subscription_cloud_;
  rclcpp::CallbackGroup::SharedPtr callback_group_cloud;

  rclcpp::Subscription<autonomous_interfaces::msg::Fiducials>::SharedPtr subscription_fiducials_;
  rclcpp::CallbackGroup::SharedPtr callback_group_fiducials;

  // Particles
  std::vector<Eigen::Matrix4f> particles;
  std::vector<float> weights;

  void allocateMemory() {
    map_octree = std::shared_ptr<octomap::OcTree>(new octomap::OcTree(0.1));

    map_tform_body_init = Eigen::Matrix4f::Identity();

    input_all_points.reset(new pcl::PointCloud<pcl::PointXYZI>());

    input_robot_odometry = Eigen::Matrix4f::Identity();

    last_robot_odometry = Eigen::Matrix4f::Identity();

    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  }

  void loadMap() {
    std::string octree_path = std::string("/ros2_ws/data") + std::string("/map.bt");
    map_octree->readBinary(octree_path);

    double x,y,z;
    map_octree->getMetricMin(x,y,z);
    octomap::point3d min(x,y,z);
    map_octree->getMetricMax(x,y,z);
    octomap::point3d max(x,y,z);

    map_edt = std::shared_ptr<DynamicEDTOctomap>(new DynamicEDTOctomap(1.0, &(*map_octree), min, max, false));
    map_edt->update();

    std::ifstream input_file_fiducials(std::string("/ros2_ws/data") + std::string("/fiducials.txt"));

    int nbr;
    input_file_fiducials >> nbr;

    RCLCPP_INFO(this->get_logger(), "Nbr of tags : %d", nbr );

    int tag_id;
    Eigen::Matrix4f map_tform_fiducial;

    for (int i = 0; i < nbr; ++i) {

       input_file_fiducials >> tag_id;

      for (int m = 0; m < 4; ++m) {
        for (int n = 0; n < 4; ++n) {
          input_file_fiducials >> map_tform_fiducial(m, n);
        }
      }

      tag_ids.insert(tag_id);
      map_tform_fiducials[tag_id] = map_tform_fiducial;

      RCLCPP_INFO(this->get_logger(), "Tag_id : %d", tag_id);

      std::cout << map_tform_fiducial << "\n";
    }

    map_loaded = true;
  }

  void fiducialsHandler(const autonomous_interfaces::msg::Fiducials::SharedPtr fiducials_msg) {

    if (!map_loaded || initialized || initial_estimation) {
      return;
    }

    int nbr_fiducials = fiducials_msg->nbr;
    for (int i = 0; i < nbr_fiducials; ++i) {

      int current_id = fiducials_msg->fiducials[i].tag_id;
      Eigen::Matrix4f current_body_tform_fiducial = poseToMatrix(fiducials_msg->fiducials[i].pose);

      if (tag_ids.find(current_id) != tag_ids.end()) {

        RCLCPP_INFO(this->get_logger(), "Found : %d", current_id);

        map_tform_body_init = map_tform_fiducials[current_id] * inverseTransformation(current_body_tform_fiducial);
        initial_estimation = true;
      }

    }

  }

  void cloudHandler(const slam::msg::Cloud::SharedPtr cloud_msg) {

    if (!map_loaded || !initial_estimation) {

      RCLCPP_INFO(this->get_logger(), "Map not loaded or no apriltag found !");

      return;
    }

    RCLCPP_INFO(this->get_logger(), "Point cloud received : %d", n);

    // Save input
    pcl::fromROSMsg(cloud_msg->cloud_deskewed, *input_all_points);

    Eigen::Quaternionf rot(cloud_msg->initial_guess_rot_w, cloud_msg->initial_guess_rot_x, cloud_msg->initial_guess_rot_y, cloud_msg->initial_guess_rot_z);
    Eigen::Vector3f trans(cloud_msg->initial_guess_x, cloud_msg->initial_guess_y, cloud_msg->initial_guess_z);

    input_robot_odometry = Eigen::Matrix4f::Identity();
    input_robot_odometry.block<3, 1>(0, 3) = trans;
    input_robot_odometry.block<3, 3>(0, 0) = rot.toRotationMatrix();

    if (!initialized) {
      initializeParticles();
      initialized = true;
    } else {
      predict(getDifferenceTransformation(last_robot_odometry, input_robot_odometry));
    }
    last_robot_odometry = input_robot_odometry;

    correct(input_all_points);

    Eigen::Matrix4f best_particles = Eigen::Matrix4f::Identity();

    float max_weight = -std::numeric_limits<float>::max();
    for (int i = 0; i < n_particles; ++i) {
      if (max_weight < weights[i]) {
        max_weight = weights[i];
        best_particles = particles[i];
      }
    }

    resample();

    RCLCPP_INFO(this->get_logger(), "Position : %f, %f, %f", best_particles(0, 3), best_particles(1, 3), best_particles(2, 3));

    ++n;

  }

  void initializeParticles() {
    particles.clear();
    weights.clear();

    for (int i = 0; i < n_particles; ++i) {
      particles.push_back(map_tform_body_init * generateRandomPose(0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
      weights.push_back(1.0 / n_particles);
    }
  }

  Eigen::Matrix4f generateRandomPose(float std_x, float std_y, float std_z, float std_roll, float std_pitch, float std_yaw) {
    std::random_device rd{};
    std::mt19937 gen{rd()};

    std::normal_distribution<float> d_x{0.0, std_x};
    std::normal_distribution<float> d_y{0.0, std_y};
    std::normal_distribution<float> d_z{0.0, std_z};
    std::normal_distribution<float> d_roll{0.0, std_roll};
    std::normal_distribution<float> d_pitch{0.0, std_pitch};
    std::normal_distribution<float> d_yaw{0.0, std_yaw};

    float x = d_x(gen);
    float y = d_y(gen);
    float z = d_z(gen);
    float roll = d_roll(gen);
    float pitch = d_pitch(gen);
    float yaw = d_yaw(gen);

    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose(0, 3) = x;
    pose(1, 3) = y;
    pose(2, 3) = z;

    return pose;
  }

  void predict(Eigen::Matrix4f displacement) {
    float dist = displacement.block<3, 1>(0, 3).norm();
    for (int i = 0; i < n_particles; ++i) {
      particles[i] *= displacement * generateRandomPose(0.01, 0.01, 0.01, 0.05, 0.05, 0.05); // Check if this is ok for the values of noise
    }
  }

  void correct(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    voxel_grid.setInputCloud(cloud); // Reduce number of ray
    voxel_grid.filter(*cloud);

    float coeff_1 = - std::log(sigma_hit) - std::log(std::sqrt(2 * M_PI));
    float coeff_2 = - 1.0 / (2.0 * sigma_hit * sigma_hit);
    float min_log_p = std::log(z_rand / max_range);

    for (int i = 0; i < n_particles; ++i) { // Compute the weights

      // Transform point cloud to be able to perform directions
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::transformPointCloud(*cloud, *cloud_trans, particles[i]);

      octomap::point3d origin(particles[i](0, 3), particles[i](1, 3), particles[i](2, 3));

      float w = 0;

      for (int j = 0; j < cloud_trans->points.size(); ++j) {

        float range = (particles[i](0, 3) - cloud_trans->points[j].x) * (particles[i](0, 3) - cloud_trans->points[j].x) + (particles[i](1, 3) - cloud_trans->points[j].y) * (particles[i](1, 3) - cloud_trans->points[j].y) + (particles[i](2, 3) - cloud_trans->points[j].z) * (particles[i](2, 3) - cloud_trans->points[j].z);

        if (range > max_range * max_range) {
          continue;
        }

        octomap::point3d endPoint(cloud_trans->points[j].x, cloud_trans->points[j].y, cloud_trans->points[j].z);
        float dist = map_edt->getDistance(endPoint);

        if (dist > 0.0){ // endpoint is inside map:
          float log_p = coeff_1 + coeff_2 * (dist * dist); 
          if (log_p < min_log_p) {
            w += min_log_p;
          } else {
            w += log_p;
          }
        } 
      }
      weights[i] = w;
    }
  }

  void resample() {
    // Normalize weigths
    float max_weight = -std::numeric_limits<float>::max();

    for (int i = 0; i < weights.size(); ++i) {
      if (weights[i] > max_weight) {
        max_weight = weights[i];
      }
    }

    for (int i = 0; i < weights.size(); ++i) {
      weights[i] = std::exp(weights[i] - max_weight);
    }

    float total_weights = 0.0;
    for (int i = 0; i < n_particles; ++i) {
      total_weights += weights[i];
    }

    for (int i = 0; i < n_particles; ++i) {
      weights[i] /= total_weights;
    }

    float interval = 1.0 / n_particles;

    float r = interval * generateRandomNumber();

    double c = weights[0];
    std::vector<int> indices;

    int n = 0;
    for (int i = 0; i < n_particles; ++i){
      float u = r + i * interval;
      while (u > c && n < n_particles){
        n = n + 1;
        c = c + weights[n];
      }
      indices.push_back(n);
    }

    std::vector<Eigen::Matrix4f> old_particles;
    old_particles = particles;

    particles.clear();
    weights.clear();

    // Particle generation
    for (int i = 0; i < n_particles; ++i) {
      particles.push_back(old_particles[indices[i]]);
      weights.push_back(1.0 / n_particles);
    }
  }

  float generateRandomNumber() {
    std::random_device rd{};
    std::mt19937 gen{rd()};

    std::uniform_real_distribution<float> dis(0.0, 1.0);

    return dis(gen);
  }

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 1, false, std::chrono::milliseconds(500));

  auto MCL = std::make_shared<MonteCarloLocalization>();
  exec.add_node(MCL);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Monte Carlo Localization Started.\033[0m");

  exec.spin();

  rclcpp::shutdown();

  return 0;
}