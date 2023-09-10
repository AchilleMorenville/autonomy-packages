#include "aut_local_planner/hybrid_a_star.h"

#include <ctime>

#include <utility>
#include <cmath>
#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "aut_utils/utils.h"
#include "aut_local_planner/dijkstra.h"

namespace aut_local_planner {

// Motion Primitives
MotionPrimitive::MotionPrimitive() : dx(0.0f), dy(0.0f), dtheta(0.0f) {}
MotionPrimitive::MotionPrimitive(float dx, float dy, float dtheta) 
    : dx(dx), dy(dy), dtheta(dtheta) {}

MotionPrimitive::MotionPrimitive(const MotionPrimitive& mp) 
    : dx(mp.dx), dy(mp.dy), dtheta(mp.dtheta) {}

MotionPrimitive& MotionPrimitive::operator=(MotionPrimitive mp) {
  std::swap(dx, mp.dx);
  std::swap(dy, mp.dy);
  std::swap(dtheta, mp.dtheta);
  return *this;
}

MotionPrimitive::~MotionPrimitive() {}

// Constructors
HybridAStar::HybridAStar() 
    : precise_target_(false), min_dist_(0.25f), target_pose_() {
  grid_to_base_link_ = Eigen::Matrix4f::Identity();
  map_to_base_link_ = Eigen::Matrix4f::Identity();
  map_to_grid_ = Eigen::Matrix4f::Identity();
  map_to_target_ = Eigen::Matrix4f::Identity();

  cost_forward_ = 1.0;
  cost_backward_ = 2.0;
  cost_side_ = 1.5;
  cost_standing_ = 3.0;
  cost_turning_ = 1.0;
  cost_change_of_direction_ = 0.1;

  int n_primitives = 10;
  std::vector<float> dxs{0.03f, 0.0f, -0.03f, 0.0f, 0.0f, 0.0f, 0.029962f, 0.029962f, -0.029962f, -0.029962f};
  std::vector<float> dys{0.0f, -0.03f, 0.0f, 0.03f, 0.0f, 0.0f, 0.001308f, -0.001308f, 0.001308f, -0.001308f};
  std::vector<float> dthetas{0.0f, 0.0f, 0.0f, 0.0f, 2 * M_PI / 72.0f, -2 * M_PI / 72.0f, -2 * M_PI / 72.0f, 2 * M_PI / 72.0f, 2 * M_PI / 72.0f, -2 * M_PI / 72.0f};
  for (int i = 0; i < n_primitives; ++i) {
    motion_primitives_.push_back(MotionPrimitive(dxs[i], dys[i], dthetas[i]));
  }
}

// Public
void HybridAStar::SetGrid(int x_dim, int y_dim, float cell_size, int theta_dim,
                          const std::vector<float>& grid, 
                          const Eigen::Matrix4f& grid_to_base_link, 
                          const Eigen::Matrix4f& map_to_base_link) {
  x_dim_ = x_dim;
  y_dim_ = y_dim;
  cell_size_ = cell_size;
  theta_dim_ = theta_dim;
  grid_ = grid;
  grid_to_base_link_ = grid_to_base_link;
  map_to_base_link_ = map_to_base_link;
  map_to_grid_ = map_to_base_link_ * aut_utils::InverseTransformation(grid_to_base_link_);
}

bool HybridAStar::SetTarget(const Eigen::Matrix4f& map_to_target) {
  Eigen::Matrix4f grid_to_target = aut_utils::InverseTransformation(map_to_grid_) * map_to_target;
  target_pose_ = Pose(grid_to_target);
  target_pose_.theta += 0.01f; // Attention
  std::cout << "Target : " << target_pose_.x << ", " << target_pose_.y << ", " << target_pose_.theta * 180.0 / M_PI << "\n";

  if (!InsideGrid(target_pose_)) { return false; }
  if (!IsFree(target_pose_)) { return false; }
  return true;
}

bool HybridAStar::GetMotion(MotionPrimitive& motion) {
  (void)motion;

  PrecomputeDijkstraHeuristic();

  HybridPriorityQueue open_set;
  std::unordered_map<int, State> closed_set;

  Eigen::Matrix4f grid_to_start = aut_utils::InverseTransformation(map_to_grid_) * map_to_base_link_;
  Pose start_pose(grid_to_start);
  start_pose.theta += 0.01f; // Attention
  float h_start = GetDijkstraHeuristic(start_pose);
  if (h_start < 0.0f) { return false; }

  // std::cout << "-------- Start Hybrid A* ---------\n";

  int count = 1;

  State start(start_pose, GetIndex(start_pose), -1, MotionPrimitive(), 
              0.0f, h_start);
  open_set.push(start);
  while (!open_set.empty()) {
    State current = open_set.top();
    open_set.pop();
    if (closed_set.find(current.id) != closed_set.end()) { continue; }
    closed_set[current.id] = current;

    // std::cout << "PQ size : " << open_set.size() << "\n";
    // std::cout << "Current : " << current.id << ", " << current.parent_id << ", " << current.g << ", "<< current.h << "\n";
    // std::cout << "          " << current.pose.x << ", " << current.pose.y << ", " << current.pose.theta * 180.0 / M_PI << "\n";

    if (IsTarget(current)) {
      // TODO: End
      std::cout << "Current : " << current.id << ", " << current.parent_id << ", " << current.g << ", "<< current.h << "\n";
      std::cout << "          " << current.pose.x << ", " << current.pose.y << ", " << current.pose.theta * 180.0 / M_PI << "\n";

      std::vector<State> path{current};
      State walker = current;
      while (walker.parent_id >= 0) {
        walker = closed_set[walker.parent_id];
        path.push_back(walker);
      }
      std::reverse(path.begin(), path.end());

      for (int i = 0; i < path.size(); ++i) {
        std::cout << "Step " << i << "\n";
        std::cout << path[i].id << ", " << path[i].parent_id << ", " << path[i].g << ", "<< path[i].h << "\n";
        std::cout << path[i].pose.x << ", " << path[i].pose.y << ", " << path[i].pose.theta * 180.0 / M_PI << "\n";
      }

      return true;
    }

    std::vector<State> neighbors;
    GetNeighbors(current, neighbors);

    // std::cout << "Neighbors size : " << neighbors.size() << "\n";
    for (State& neighbor : neighbors) {

      // std::cout << "Neigbor : " << neighbor.id << ", " << neighbor.parent_id << ", " << neighbor.g << ", "<< neighbor.h << "\n";
      // std::cout << "          " << neighbor.pose.x << ", " << neighbor.pose.y << ", " << neighbor.pose.theta * 180.0 / M_PI << "\n";
      if (closed_set.find(neighbor.id) != closed_set.end()) { continue; }
      open_set.push(neighbor);
      count++;
    }
  }
  // std::cout << count << "\n";
  return false;
}

// Private
HybridAStar::Pose::Pose() : x(0.0), y(0.0), theta(0.0) {}

HybridAStar::Pose::Pose(const Eigen::Matrix4f& grid_to_pose) {
  x = grid_to_pose(0, 3);
  y = grid_to_pose(1, 3);
  Eigen::Matrix4f x_axis = Eigen::Matrix4f::Identity();
  x_axis(0, 3) = 1.0f;
  Eigen::Matrix4f grid_to_x_axis = grid_to_pose * x_axis;
  float d_x = grid_to_x_axis(0, 3) - x;
  float d_y = grid_to_x_axis(1, 3) - y;
  theta = std::atan2(d_y, d_x);
}

HybridAStar::Pose::Pose(float x, float y, float theta) 
    : x(x), y(y), theta(theta) {}

HybridAStar::Pose::Pose(const Pose& p) : x(p.x), y(p.y), theta(p.theta) {}

HybridAStar::Pose& HybridAStar::Pose::operator=(Pose p) {
  std::swap(x, p.x);
  std::swap(y, p.y);
  std::swap(theta, p.theta);
  return *this;
}

HybridAStar::Pose HybridAStar::Pose::operator+(const MotionPrimitive& motion_primitive) const {
  Eigen::Matrix2f r_pose;
  r_pose << std::cos(theta), -std::sin(theta),
            std::sin(theta),  std::cos(theta);
  Eigen::Vector2f t_pose(x, y);
  Eigen::Matrix3f m_pose = Eigen::Matrix3f::Identity();
  m_pose.block<2, 2>(0, 0) = r_pose;
  m_pose.block<2, 1>(0, 2) = t_pose;

  Eigen::Matrix2f r_mp;
  r_mp << std::cos(motion_primitive.dtheta), -std::sin(motion_primitive.dtheta),
          std::sin(motion_primitive.dtheta),  std::cos(motion_primitive.dtheta);
  Eigen::Vector2f t_mp(motion_primitive.dx, motion_primitive.dy);
  Eigen::Matrix3f m_mp = Eigen::Matrix3f::Identity();
  m_mp.block<2, 2>(0, 0) = r_mp;
  m_mp.block<2, 1>(0, 2) = t_mp;
  Eigen::Matrix3f new_pose = m_pose * m_mp;
  return Pose(new_pose(0, 2), new_pose(1, 2), 
              Eigen::Rotation2Df(new_pose.block<2, 2>(0, 0)).angle());
}

HybridAStar::Pose::~Pose() {}

HybridAStar::State::State() 
    : pose(), id(), parent_id(), motion_from_parent(), g(), h() {}

HybridAStar::State::State(Pose pose, int id, int parent_id, 
                          MotionPrimitive motion_from_parent, float g, 
                          float h) 
    : pose(pose), 
      id(id), 
      parent_id(parent_id), 
      motion_from_parent(motion_from_parent), 
      g(g), 
      h(h) {}

HybridAStar::State::State(const State& s) 
    : pose(s.pose), 
      id(s.id), 
      parent_id(s.parent_id), 
      motion_from_parent(s.motion_from_parent), 
      g(s.g), 
      h(s.h) {}


HybridAStar::State& HybridAStar::State::operator=(State s) {
  pose = s.pose;
  std::swap(id, s.id);
  std::swap(parent_id, s.parent_id);
  motion_from_parent = s.motion_from_parent;
  std::swap(g, s.g);
  std::swap(h, s.h);
  return *this;
}

HybridAStar::State::~State() {}

bool HybridAStar::InsideGrid(const Pose& pose) {
  int idx_x = static_cast<int>(pose.x / cell_size_);
  int idx_y = static_cast<int>(pose.y / cell_size_);
  return idx_x >= 0 && idx_x < x_dim_ && idx_y >= 0 && idx_y < y_dim_;
}

bool HybridAStar::IsFree(const Pose& pose) {
  int idx_x = static_cast<int>(pose.x / cell_size_);
  int idx_y = static_cast<int>(pose.y / cell_size_);
  int idx = idx_x + x_dim_ * idx_y;
  return grid_[idx] >= min_dist_;
}

int HybridAStar::GetIndex(const Pose& pose) {
  int idx_x = static_cast<int>(pose.x / cell_size_);
  int idx_y = static_cast<int>(pose.y / cell_size_);
  // int idx_theta = static_cast<int>((pose.theta + M_PI) * theta_dim_ / (2 * M_PI));
  int idx_theta = static_cast<int>((pose.theta * theta_dim_) / (2.0f * M_PI) + theta_dim_ / 2.0f);
  if (idx_x < 0 || idx_x >= x_dim_ || 
      idx_y < 0 || idx_y >= y_dim_ || 
      idx_theta < 0 || idx_theta >= theta_dim_) {
    return -1;
  }
  return idx_x + x_dim_ * idx_y + (x_dim_ * y_dim_) * idx_theta;
}

int HybridAStar::GetIndex2D(const Pose& pose) {
  int idx_x = static_cast<int>(pose.x / cell_size_);
  int idx_y = static_cast<int>(pose.y / cell_size_);
  return idx_x + x_dim_ * idx_y;
}

void HybridAStar::PrecomputeDijkstraHeuristic() {
  Dijkstra d(x_dim_, y_dim_, cell_size_, min_dist_);
  d.Compute(target_pose_.x, target_pose_.y, grid_, dijkstra_grid_);

  // std::cout.precision(3);
  // std::cout << std::fixed;
  // std::cout << "------- Dijkstra -------\n";
  // for (int i = 0; i < x_dim_; ++i) {
  //   for (int j = 0; j < y_dim_; ++j) {
  //     std::cout << dijkstra_grid_[i + x_dim_ * j] << ", ";
  //   }
  //   std::cout << "\n";
  // }
  // std::cout << "------------------------\n";

}

float HybridAStar::GetDijkstraHeuristic(const Pose& pose) {
  int idx_x = static_cast<int>(pose.x / cell_size_);
  int idx_y = static_cast<int>(pose.y / cell_size_);
  return dijkstra_grid_[idx_x + x_dim_ * idx_y];
}

bool HybridAStar::IsTarget(const State& state) {
  if (precise_target_) {
    return state.id == GetIndex(target_pose_);
  } else {
    return GetIndex2D(state.pose) == GetIndex2D(target_pose_);
  }
}

void HybridAStar::GetNeighbors(const State& state, std::vector<State>& neighbors) {
  for (MotionPrimitive& motion_primitive : motion_primitives_) {
    Pose neighbor_pose = state.pose + motion_primitive;
    if (!Accessible(neighbor_pose)) { continue; }
    float neighbor_g = state.g + GetCost(neighbor_pose, state.motion_from_parent, motion_primitive);
    float neighbor_h = GetDijkstraHeuristic(neighbor_pose);
    if (neighbor_h < 0.0f) { continue; }
    State neighbor_state(neighbor_pose, GetIndex(neighbor_pose), state.id, 
                         motion_primitive, neighbor_g, 
                         neighbor_h);
    neighbors.push_back(neighbor_state);
  }
}

bool HybridAStar::Accessible(const Pose& pose) {
  if (!InsideGrid(pose) || !IsFree(pose) || GetIndex(pose) < 0) { return false; }
  return true; // TODO: Check collisions;
}

float HybridAStar::GetCost(const Pose& pose, const MotionPrimitive& previous_motion, const MotionPrimitive& current_motion) {
  if (current_motion.dx != 0.0f && current_motion.dy != 0.0f) {  // Walking
    if (current_motion.dx > 0.0f) {  // Forward
      if (current_motion.dtheta == 0.0f) {  // Not turning
        return cell_size_ * cost_forward_;
      } else {  // Turning
        if ((0.0f < previous_motion.dtheta) - (previous_motion.dtheta < 0.0f) !=
            (0.0f < current_motion.dtheta) - (current_motion.dtheta < 0.0f)) {  // Change of direction
          return cell_size_ * (cost_forward_ + cost_turning_ + cost_change_of_direction_);
        } else {  // Stay in direction
          return cell_size_ * (cost_forward_ + cost_turning_);
        }
      }
    } else if (current_motion.dx < 0.0f) {  // Backward
      if (current_motion.dtheta == 0.0f) {  // Not turning
        return cell_size_ * cost_backward_;
      } else {  // Turning
        if ((0.0f < previous_motion.dtheta) - (previous_motion.dtheta < 0.0f) !=
            (0.0f < current_motion.dtheta) - (current_motion.dtheta < 0.0f)) {  // Change of direction
          return cell_size_ * (cost_backward_ + cost_turning_ + cost_change_of_direction_);
        } else {  // Stay in direction
          return cell_size_ * (cost_backward_ + cost_turning_);
        }
      }
    } else {  // Side
      return cell_size_ * cost_side_;
    }
  } else {  // Standing
    if ((0.0f < previous_motion.dtheta) - (previous_motion.dtheta < 0.0f) !=
        (0.0f < current_motion.dtheta) - (current_motion.dtheta < 0.0f)) {  // Change of direction
      return cell_size_ * (cost_standing_ + cost_turning_ + cost_change_of_direction_);
    } else {  // Stay in direction
      return cell_size_ * (cost_standing_ + cost_turning_);
    }
  }
  return cell_size_;
}

}  // aut_local_planner

int main(int argc, char * argv[]) {

  aut_local_planner::HybridAStar has;

  Eigen::Matrix4f map_to_base_link = Eigen::Matrix4f::Identity();
  map_to_base_link(0, 3) = 0.2f;
  map_to_base_link(1, 3) = 0.2f;

  std::vector<float> grid(128 * 128, 1.0f);
  has.SetGrid(128, 128, 0.03f, 72, grid, map_to_base_link, map_to_base_link);
  
  Eigen::Matrix4f map_to_target = Eigen::Matrix4f::Identity();
  map_to_target(0, 3) = 3.0f;
  map_to_target(1, 3) = 3.0f;
  bool set_traget = has.SetTarget(map_to_target);

  if (set_traget) {
    aut_local_planner::MotionPrimitive mp;

    std::clock_t t_start = std::clock();
    bool result = has.GetMotion(mp);
    std::cout << "Time taken : " << static_cast<double>(1000.0 * (std::clock() - t_start) / CLOCKS_PER_SEC) << "ms\n";
    std::cout << "Result: " << result << "\n";
  }

  // aut_local_planner::Dijkstra d(3, 3, 0.3f, 0.25f);

  // std::vector<float> dijkstra_grid;
  // std::vector<float> test1 { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
  // d.Compute(0.1f, 0.1f, test1, dijkstra_grid);
  // PrintResult(dijkstra_grid, 3, 3);

  // std::vector<float> test2 { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
  // d.Compute(0.45f, 0.45f, test2, dijkstra_grid);
  // PrintResult(dijkstra_grid, 3, 3);

  // std::vector<float> test3 { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
  // d.Compute(0.1f, 0.45f, test3, dijkstra_grid);
  // PrintResult(dijkstra_grid, 3, 3);

  // std::vector<float> test4 { 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 };
  // d.Compute(0.1f, 0.1f, test4, dijkstra_grid);
  // PrintResult(dijkstra_grid, 3, 3);

  // std::vector<float> test5 { 1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0 };
  // d.Compute(0.1f, 0.1f, test5, dijkstra_grid);
  // PrintResult(dijkstra_grid, 3, 3);

  // std::vector<float> test6 { 1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0 };
  // d.Compute(1.0f, 1.0f, test6, dijkstra_grid);
  // PrintResult(dijkstra_grid, 3, 3);

}