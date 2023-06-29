#include "aut_local_planner/local_grid.h"

#include <vector>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <tuple>
#include <cmath>
#include <iostream>

#include <Eigen/Core>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <aut_utils/utils.h>

namespace aut_local_planner {

LocalGrid::LocalGrid(float min_dist, float max_dist, float min_dist_to_target, float obstacle_coeff) {
  min_dist_ = min_dist;
  max_dist_ = max_dist;
  min_dist_to_target_ = min_dist_to_target;
  obstacle_coeff_ = obstacle_coeff;
}

void LocalGrid::AddLocalGrid(std::vector<float> local_grid, Eigen::Matrix4f map_to_base_link, Eigen::Matrix4f base_link_to_local_grid) {
  local_grid_ = local_grid;
  map_to_base_link_ = map_to_base_link;
  base_link_to_local_grid_ = base_link_to_local_grid;
}

bool LocalGrid::IsInGrid(Eigen::Vector3f position) {
  Eigen::Matrix4f map_to_position = Eigen::Matrix4f::Identity();
  map_to_position.block<3, 1>(0, 3) = position;
  Eigen::Matrix4f local_grid_to_position = aut_utils::InverseTransformation(map_to_base_link_ * base_link_to_local_grid_) * map_to_position;
  return IsInGrid(std::pair<int, int>((int) (local_grid_to_position(0, 3) / 0.03f), (int) (local_grid_to_position(1, 3) / 0.03f)));
}

bool LocalGrid::IsCurrentPositionRotationSafe() {
  Eigen::Matrix4f local_grid_to_base_link = aut_utils::InverseTransformation(base_link_to_local_grid_);
  std::pair<int, int> start_indexes((int) (local_grid_to_base_link(0, 3) / 0.03f), (int) (local_grid_to_base_link(1, 3) / 0.03f));
  if (!IsInGrid(start_indexes)) {
    return false;
  }
  return local_grid_[start_indexes.first + 128 * start_indexes.second] >= max_dist_;
}

int LocalGrid::GetDirection(std::vector<Eigen::Vector3f> targets_position, Eigen::Vector2f& direction) {

  // Compute position of target and start in local grid.
  Eigen::Matrix4f local_grid_to_base_link = aut_utils::InverseTransformation(base_link_to_local_grid_);
  std::pair<int, int> start_indexes((int) (local_grid_to_base_link(0, 3) / 0.03f), (int) (local_grid_to_base_link(1, 3) / 0.03f));

  for (int i = targets_position.size() - 1; i >=0 ; --i) {

    std::cout << "Try target nbr: " << i << "\n";

    Eigen::Matrix4f map_to_target = Eigen::Matrix4f::Identity();
    map_to_target.block<3, 1>(0, 3) = targets_position[i];
    Eigen::Matrix4f local_grid_to_target = aut_utils::InverseTransformation(map_to_base_link_ * base_link_to_local_grid_) * map_to_target;

    std::pair<int, int> target_indexes((int) (local_grid_to_target(0, 3) / 0.03f), (int) (local_grid_to_target(1, 3) / 0.03f));
    std::cout << "Compute Heuristic\n";

    // Compute dijkstra heuristic for the A* search
    std::vector<float> h_local_grid;
    ComputeHeuristic(target_indexes, h_local_grid);

    if (h_local_grid[start_indexes.first + 128 * start_indexes.second] < 0.0f) { // Taget is not accessible with dijkstra
      std::cout << "The target is not accessible from the start\n";
      continue;
    }

    if ((map_to_base_link_.block<3, 1>(0, 3) - targets_position[i]).norm() < min_dist_to_target_) {
      direction(0) = 0.0f;
      direction(1) = 0.0f;
      return i;
    }

    std::cout << "Start A*\n";
    std::vector<std::pair<int, int>> path;
    bool found_path = FindPath(start_indexes, target_indexes, h_local_grid, path);

    current_path_ = path;

    if (!found_path) {
      continue;
    }

    std::pair<int, int> point_to_follow = GetTarget(path);

    Eigen::Matrix4f x_axis = Eigen::Matrix4f::Identity();
    x_axis(0, 3) = 1.0f;

    Eigen::Matrix4f local_grid_to_x_axis = local_grid_to_base_link * x_axis;
    std::pair<int, int> indexes_x_axis((int) (local_grid_to_x_axis(0, 3) / 0.03f), (int) (local_grid_to_x_axis(1, 3) / 0.03f));

    Eigen::Vector2f vector_base_link_x;
    vector_base_link_x(0) = (indexes_x_axis.first - start_indexes.first);
    vector_base_link_x(1) = (indexes_x_axis.second - start_indexes.second);
    vector_base_link_x /= vector_base_link_x.norm();

    Eigen::Matrix4f y_axis = Eigen::Matrix4f::Identity();
    y_axis(1, 3) = 1.0f;

    Eigen::Matrix4f local_grid_to_y_axis = local_grid_to_base_link * y_axis;
    std::pair<int, int> indexes_y_axis((int) (local_grid_to_y_axis(0, 3) / 0.03f), (int) (local_grid_to_y_axis(1, 3) / 0.03f));

    Eigen::Vector2f vector_base_link_y;
    vector_base_link_y(0) = (indexes_y_axis.first - start_indexes.first);
    vector_base_link_y(1) = (indexes_y_axis.second - start_indexes.second);
    vector_base_link_y /= vector_base_link_y.norm();

    Eigen::Vector2f vector_target;
    vector_target(0) = (point_to_follow.first - start_indexes.first);
    vector_target(1) = (point_to_follow.second - start_indexes.second);

    direction(0) = vector_target.dot(vector_base_link_x) * 0.03f;
    direction(1) = vector_target.dot(vector_base_link_y) * 0.03f;

    // Eigen::Matrix4f local_grid_to_base_link = aut_utils::InverseTransformation(base_link_to_local_grid_);
    // Eigen::Matrix4f base_link_to_x_axis = Eigen::Matrix4f::Identity();
    // base_link_to_x_axis(0, 3) = 1.0f;
    // Eigen::Matrix4f local_grid_to_x_axis = local_grid_to_base_link * base_link_to_x_axis;

    // float current_dx = local_grid_to_x_axis(0, 3) - local_grid_to_base_link(0, 3);
    // float current_dy = local_grid_to_x_axis(1, 3) - local_grid_to_base_link(1, 3);

    // Eigen::Vector2f projected_x(current_dx, current_dy);
    // Eigen::Vector2f projected_y(-current_dy, current_dx);
    
    // projected_x.normalize();
    // projected_y.normalize();

    // std::pair<int, int> point = path[std::min(5,(int) path.size())];
    // Eigen::Matrix4f local_grid_to_point = Eigen::Matrix4f::Identity();
    // local_grid_to_point(0, 3) = (point.first + 0.5f) * 0.03f;
    // local_grid_to_point(1, 3) = (point.second + 0.5f) * 0.03f;

    // Eigen::Vector2f vector_point(local_grid_to_point(0, 3) - local_grid_to_base_link(0, 3), local_grid_to_point(1, 3) - local_grid_to_base_link(1, 3));
    // float direction_dist = vector_point.norm();
    // // vector_point.normalize();

    // float dx = vector_point.dot(projected_x);
    // float dy = vector_point.dot(projected_y);

    // direction(0) = dx;
    // direction(1) = dy;

    if (found_path) {
      std::cout << "Found path " << i << " on " << targets_position.size() << "\n";
      return i;
    }

  }

  return -1;
}

std::pair<int, int> LocalGrid::GetTarget(std::vector<std::pair<int, int>> &path) {
  std::pair<int, int> start = path[0];
  int best_id = -1;
  for (int best = path.size() - 1; best >= 0; --best) {
    float max_dist_from_line = -1;
    for (int i = 0; i <= best; ++i) {
      float dist = std::abs((path[best].first - start.first) * (start.second - path[i].second) - (start.first - path[i].first) * (path[best].second - start.second)) / std::sqrt((start.first - path[best].first) * (start.first - path[best].first) + (start.second - path[best].second) * (start.second - path[best].second));
      if (dist > max_dist_from_line) {
        max_dist_from_line = dist;
      }
    }
    if (max_dist_from_line < 5.0) {
      best_id = best;
      break;
    }
  }
  return path[best_id];
}

void LocalGrid::ComputeHeuristic(std::pair<int, int> target_indexes, std::vector<float>& h_local_grid) {
  // Computes Dijkstra

  std::unordered_map<std::pair<int, int>, float, hash_pair> g_score;
  g_score[target_indexes] = 0.0f;

  std::vector<std::pair<int, int>> open_set;
  open_set.push_back(target_indexes);

  while (!open_set.empty()) {
    std::vector<std::pair<int, int>>::iterator current_it = open_set.begin();
    std::pair<int, int> current = *current_it;

    for (auto it = open_set.begin(); it != open_set.end(); it++) {
      if (g_score[*it] < g_score[current]) {
        current_it = it;
        current = *it;
      }
    }

    open_set.erase(current_it);
    std::vector<std::pair<int, int>> neighbors = GetNeighbors(current);

    for (std::pair<int, int> neighbor: neighbors) {
      
      float tentative_g_score = g_score[current] + GetEuclideanDist(current, neighbor); // * (1 + obstacle_coeff_ * GetWeight(current));
      float neighbor_g_score = g_score.find(neighbor) == g_score.end() ? std::numeric_limits<float>::max() : g_score[neighbor];

      if (tentative_g_score < neighbor_g_score) { // Add it because we have found better
        g_score[neighbor] = tentative_g_score;
        std::vector<std::pair<int, int>>::iterator it = std::find(open_set.begin(), open_set.end(), neighbor);
        if (it == open_set.end()) {
          open_set.push_back(neighbor);
        }
      }
    }
  }

  for (int i = 0; i < 128 * 128; ++i) {
    std::pair<int, int> indexes(i % 128, i / 128);
    if (g_score.find(indexes) != g_score.end()) {
      h_local_grid.push_back(g_score[indexes]);
    } else {
      h_local_grid.push_back(-1.0f);
    }
  }
}

std::vector<std::pair<int, int>> LocalGrid::GetNeighbors(std::pair<int, int> p) {
  std::vector<std::pair<int, int>> neighbors;
  for (int i = -1; i < 2; i++) {
    for (int j = -1; j < 2; j++) {
      if (i == 0 && j == 0) {
        continue;
      }
      std::pair<int, int> neighbor(p.first + i, p.second + j);
      if (!IsFree(neighbor)) {
        continue;
      }
      neighbors.push_back(neighbor);
    }
  }
  return neighbors;
}

bool LocalGrid::IsInGrid(std::pair<int, int> p) {
  return p.first >= 0 && p.first < 128 && p.second >= 0 && p.second < 128;
}

bool LocalGrid::IsFree(std::pair<int, int> p) {
  if (!IsInGrid(p)) {
    return false;
  }
  return local_grid_[p.first + 128 * p.second] >= min_dist_;
}

float LocalGrid::GetEuclideanDist(std::pair<int, int> p1, std::pair<int, int> p2) {
  return 0.03f * std::sqrt((p1.first - p2.first) * (p1.first - p2.first) + (p1.second - p2.second) * (p1.second - p2.second));
}

bool LocalGrid::FindPath(std::pair<int, int> start, std::pair<int, int> goal, std::vector<float>& h_local_grid, std::vector<std::pair<int, int>> &path) {

  std::unordered_map<std::pair<int, int>, std::pair<int, int>, hash_pair> came_from;

  std::unordered_map<std::pair<int, int>, float, hash_pair> f_score;
  f_score[start] = h_local_grid[start.first + 128 * start.second];

  std::unordered_map<std::pair<int, int>, float, hash_pair> g_score;
  g_score[start] = 0;

  std::vector<std::pair<int, int>> open_set;
  open_set.push_back(start);

  while (!open_set.empty()) {
    std::vector<std::pair<int, int>>::iterator current_it = open_set.begin();
    std::pair<int, int> current = *current_it;

    for (auto it = open_set.begin(); it != open_set.end(); it++) {
      if (f_score[*it] < f_score[current]) {
        current_it = it;
        current = *it;
      }
    }

    // if (current == goal) { // Is it a good idea ? Or need to do like dijkstra and find all to find best path

    //   while (came_from.find(current) != came_from.end()) {
    //     path.push_back(current);
    //     current = came_from[current];
    //   }

    //   path.push_back(start);

    //   std::reverse(path.begin(), path.end());

    //   return true;
    // }

    open_set.erase(current_it);

    std::vector<std::pair<int, int>> neighbors = GetNeighbors(current);

    for (std::pair<int, int> neighbor: neighbors) {

      float tentative_g_score = g_score[current] + GetEuclideanDist(current, neighbor) * (1 + obstacle_coeff_ * GetWeight(current)); // Change weight

      float neighbor_g_score = g_score.find(neighbor) == g_score.end() ? std::numeric_limits<float>::max() : g_score[neighbor];

      if (tentative_g_score < neighbor_g_score) { // Add it because we have found better

        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g_score;
        f_score[neighbor] = tentative_g_score + h_local_grid[neighbor.first + 128 * neighbor.second];

        std::vector<std::pair<int, int>>::iterator it = std::find(open_set.begin(), open_set.end(), neighbor);
        if (it == open_set.end()) {
          open_set.push_back(neighbor);
        }
      }
    }
  }

  if (came_from.find(goal) != came_from.end()) { // Is it a good idea ? Or need to do like dijkstra and find all to find best path

    std::pair<int, int> current = goal;

    while (came_from.find(current) != came_from.end()) {
      path.push_back(current);
      current = came_from[current];
    }

    path.push_back(start);

    std::reverse(path.begin(), path.end());

    return true;
  }

  return false;

}

float LocalGrid::GetWeight(std::pair<int, int> p) {
  float obstacle_dist = local_grid_[p.first + 128 * p.second];
  if (obstacle_dist <= min_dist_) {
    return 1.0f;
  }
  if (obstacle_dist > max_dist_) {
    return 0.0f;
  }
  float t = (obstacle_dist - min_dist_) / (max_dist_ - min_dist_); 

  return 1 - (t * t * (3 - 2 * t));
}

nav_msgs::msg::OccupancyGrid LocalGrid::LocalGridToOccupancyGrid() {

  nav_msgs::msg::OccupancyGrid occupancy_grid;

  // occupancy_grid.data.reserve(128 * 128);

  for (int i = 0; i < 128 * 128; ++i) {
    if (local_grid_[i] < 0.25f) {
      occupancy_grid.data.push_back(100);
    } else {
      occupancy_grid.data.push_back(0);
    }
  }

  for (int i = 0; i < (int) current_path_.size(); ++i) {
    occupancy_grid.data[current_path_[i].first + 128 * current_path_[i].second] = 100;
  }

  // float max_h = -std::numeric_limits<float>::max();
  // for (int i = 0; i < (int) h_local_grid.size(); ++i) {
  //   if (h_local_grid[i] > max_h) {
  //     max_h = h_local_grid[i];
  //   }
  // }

  // occupancy_grid.data.reserve(128 * 128);

  // for (int i = 0; i < 128; ++i) {
  //   for (int j = 0; j < 128; ++j) {
  //     if (local_grid_[i + 128 * j] < 0.25f) {
  //       occupancy_grid.data.push_back(100);
  //     } else {
  //       occupancy_grid.data.push_back(0);
  //     }
  //   }
  // }

  // for (int i = 0; i < 128 * 128; ++i) {
  //   if (local_grid_[i] < 0.25f) {
  //     occupancy_grid.data.push_back(100);
  //   } else {
  //     occupancy_grid.data.push_back(0);
  //   }
  // }

  occupancy_grid.header.frame_id = std::string("base_link");

  occupancy_grid.info.resolution = 0.03;
  occupancy_grid.info.width = 128;
  occupancy_grid.info.height = 128;

  geometry_msgs::msg::Pose base_link_to_local_grid_pose = aut_utils::MatrixToPose(base_link_to_local_grid_);

  occupancy_grid.info.origin.position.x = base_link_to_local_grid_pose.position.x;
  occupancy_grid.info.origin.position.y = base_link_to_local_grid_pose.position.y;
  occupancy_grid.info.origin.position.z = base_link_to_local_grid_pose.position.z;

  occupancy_grid.info.origin.orientation.x = base_link_to_local_grid_pose.orientation.x;
  occupancy_grid.info.origin.orientation.y = base_link_to_local_grid_pose.orientation.y;
  occupancy_grid.info.origin.orientation.z = base_link_to_local_grid_pose.orientation.z;
  occupancy_grid.info.origin.orientation.w = base_link_to_local_grid_pose.orientation.w;

  return occupancy_grid;
}

}  // namespace aut_local_planner


// #include "aut_local_planner/local_grid.h"

// #include <vector>
// #include <utility>
// #include <unordered_map>
// #include <unordered_set>
// #include <limits>
// #include <tuple>
// #include <cmath>
// #include <iostream>

// #include <Eigen/Core>

// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <geometry_msgs/msg/pose.hpp>

// #include <aut_utils/utils.h>

// namespace aut_local_planner {

// LocalGrid::LocalGrid(float min_dist, float min_dist_to_target) {
//   min_dist_ = min_dist;
//   min_dist_to_target_ = min_dist_to_target;

//   MotionPrimitive forward;
//   forward.x = 0.15f;
//   forward.y = 0.0f;
//   forward.angle = 0.0f;
//   forward.cost = 0.15f;

//   motion_primitives_.push_back(forward);

//   MotionPrimitive backward;
//   forward.x = -0.15f;
//   forward.y = 0.0f;
//   forward.angle = 0.0f;
//   forward.cost = 0.15f;

//   motion_primitives_.push_back(backward);

//   MotionPrimitive left;
//   forward.x = 0.0f;
//   forward.y = 0.15f;
//   forward.angle = 0.0f;
//   forward.cost = 0.15f;

//   motion_primitives_.push_back(left);

//   MotionPrimitive right;
//   forward.x = 0.0f;
//   forward.y = -0.15f;
//   forward.angle = 0.0f;
//   forward.cost = 0.15f;

//   motion_primitives_.push_back(right);

//   MotionPrimitive rotate_left;
//   forward.x = 0.0f;
//   forward.y = 0.0f;
//   forward.angle = 22.5f * M_PI / 180.0f;
//   forward.cost = 0.05f;

//   motion_primitives_.push_back(rotate_left);

//   MotionPrimitive rotate_right;
//   forward.x = 0.0f;
//   forward.y = 0.0f;
//   forward.angle = -22.5f * M_PI / 180.0f;
//   forward.cost = 0.05f;

//   motion_primitives_.push_back(rotate_right);
// }

// void LocalGrid::AddLocalGrid(std::vector<float> local_grid, Eigen::Matrix4f map_to_base_link, Eigen::Matrix4f base_link_to_local_grid) {
//   local_grid_ = local_grid;
//   map_to_base_link_ = map_to_base_link;
//   base_link_to_local_grid_ = base_link_to_local_grid;
// }

// bool LocalGrid::IsInGrid(Eigen::Vector3f position) {
//   Eigen::Matrix4f map_to_position = Eigen::Matrix4f::Identity();
//   map_to_position.block<3, 1>(0, 3) = position;
//   Eigen::Matrix4f local_grid_to_position = aut_utils::InverseTransformation(map_to_base_link_ * base_link_to_local_grid_) * map_to_position;
//   return IsInGrid(std::pair<int, int>((int) (local_grid_to_position(0, 3) / 0.03f), (int) (local_grid_to_position(1, 3) / 0.03f)));
// }

// int LocalGrid::GetCommand(std::vector<Eigen::Vector3f> targets_position) {

//   // Compute position of target and start in local grid.
//   Eigen::Matrix4f local_grid_to_base_link = aut_utils::InverseTransformation(base_link_to_local_grid_);
//   std::pair<int, int> start_indexes((int) (local_grid_to_base_link(0, 3) / 0.03f), (int) (local_grid_to_base_link(1, 3) / 0.03f));

//   for (int i = targets_position.size() - 1; i >=0 ; --i) {

//     std::cout << "Try target nbr: " << i << "\n";

//     Eigen::Matrix4f map_to_target = Eigen::Matrix4f::Identity();
//     map_to_target.block<3, 1>(0, 3) = targets_position[i];
//     Eigen::Matrix4f local_grid_to_target = aut_utils::InverseTransformation(map_to_base_link_ * base_link_to_local_grid_) * map_to_target;

//     std::pair<int, int> target_indexes((int) (local_grid_to_target(0, 3) / 0.03f), (int) (local_grid_to_target(1, 3) / 0.03f));

//     std::cout << "Compute Heuristic\n";

//     // Compute dijkstra heuristic for the A* search
//     std::vector<float> h_local_grid;
//     ComputeHeuristic(target_indexes, h_local_grid);

//     if (h_local_grid[start_indexes.first + 128 * start_indexes.second] < 0.0f) { // Taget is not accessible with dijkstra
//       std::cout << "The target is not accessible from the start\n";
//       continue;
//     }

//     std::cout << "Start hybrid\n";
//     MotionPrimitive command;
//     bool found_path = HybridPathPlanning(target_indexes, h_local_grid, command);

//     if (found_path) {
//       return i;
//     }

//   }

//   return -1;
// }

// void LocalGrid::ComputeHeuristic(std::pair<int, int> target_indexes, std::vector<float>& h_local_grid) {
//   // Computes Dijkstra

//   std::unordered_map<std::pair<int, int>, float, hash_pair> g_score;
//   g_score[target_indexes] = 0.0f;

//   std::vector<std::pair<int, int>> open_set;
//   open_set.push_back(target_indexes);

//   while (!open_set.empty()) {
//     std::vector<std::pair<int, int>>::iterator current_it = open_set.begin();
//     std::pair<int, int> current = *current_it;

//     for (auto it = open_set.begin(); it != open_set.end(); it++) {
//       if (g_score[*it] < g_score[current]) {
//         current_it = it;
//         current = *it;
//       }
//     }

//     open_set.erase(current_it);
//     std::vector<std::pair<int, int>> neighbors = GetNeighbors(current);

//     for (std::pair<int, int> neighbor: neighbors) {
      
//       float tentative_g_score = g_score[current] + GetEuclideanDist(current, neighbor); // * (1 + obstacle_coeff_ * GetWeight(current));
//       float neighbor_g_score = g_score.find(neighbor) == g_score.end() ? std::numeric_limits<float>::max() : g_score[neighbor];

//       if (tentative_g_score < neighbor_g_score) { // Add it because we have found better
//         g_score[neighbor] = tentative_g_score;
//         std::vector<std::pair<int, int>>::iterator it = std::find(open_set.begin(), open_set.end(), neighbor);
//         if (it == open_set.end()) {
//           open_set.push_back(neighbor);
//         }
//       }
//     }
//   }

//   for (int i = 0; i < 128 * 128; ++i) {
//     std::pair<int, int> indexes(i % 128, i / 128);
//     if (g_score.find(indexes) != g_score.end()) {
//       h_local_grid.push_back(g_score[indexes]);
//     } else {
//       h_local_grid.push_back(-1.0f);
//     }
//   }
// }

// std::vector<std::pair<int, int>> LocalGrid::GetNeighbors(std::pair<int, int> p) {
//   std::vector<std::pair<int, int>> neighbors;
//   for (int i = -1; i < 2; i++) {
//     for (int j = -1; j < 2; j++) {
//       if (i == 0 && j == 0) {
//         continue;
//       }
//       std::pair<int, int> neighbor(p.first + i, p.second + j);
//       if (!IsFree(neighbor)) {
//         continue;
//       }
//       neighbors.push_back(neighbor);
//     }
//   }
//   return neighbors;
// }

// bool LocalGrid::IsInGrid(std::pair<int, int> p) {
//   return p.first >= 0 && p.first < 128 && p.second >= 0 && p.second < 128;
// }

// bool LocalGrid::IsFree(std::pair<int, int> p) {
//   if (!IsInGrid(p)) {
//     return false;
//   }
//   return local_grid_[p.first + 128 * p.second] >= min_dist_;
// }

// float LocalGrid::GetEuclideanDist(std::pair<int, int> p1, std::pair<int, int> p2) {
//   return 0.03f * std::sqrt((p1.first - p2.first) * (p1.first - p2.first) + (p1.second - p2.second) * (p1.second - p2.second));
// }

// bool LocalGrid::HybridPathPlanning(const std::pair<int, int> target_indexes, const std::vector<float>& h_local_grid, MotionPrimitive& command) {

//   std::cout << "Get StateNode for start\n";

//   // Get starting state
//   Eigen::Matrix4f local_grid_to_base_link = aut_utils::InverseTransformation(base_link_to_local_grid_);
//   Eigen::Matrix4f base_link_to_x_axis = Eigen::Matrix4f::Identity();
//   base_link_to_x_axis(0, 3) = 1.0f;
//   Eigen::Matrix4f local_grid_to_x_axis = local_grid_to_base_link * base_link_to_x_axis;

//   float dx = local_grid_to_x_axis(0, 3) - local_grid_to_base_link(0, 3);
//   float dy = local_grid_to_x_axis(1, 3) - local_grid_to_base_link(1, 3);
//   float heading = std::atan2(dy, dx);

//   std::shared_ptr<StateNode> start = std::make_shared<StateNode>();
//   start->x = local_grid_to_base_link(0, 3);
//   start->y = local_grid_to_base_link(1, 3);
//   start->theta = heading;
//   start->g = 0;
//   start->f = GetHeuristicFromPose(*start, h_local_grid);

//   std::tuple<int, int, int> start_triplet = GetTriplet(*start);

//   std::vector<std::tuple<int, int, int>> open_set;
//   std::unordered_set<std::tuple<int, int, int>, hash_triplet> closed_set;
//   std::unordered_map<std::tuple<int, int, int>, std::shared_ptr<StateNode>, hash_triplet> dicretize_grid;

//   open_set.push_back(start_triplet);
//   dicretize_grid[start_triplet] = start;

//   while (!open_set.empty()) {
//     std::vector<std::tuple<int, int, int>>::iterator current_it = open_set.begin();
//     std::tuple<int, int, int> current_triplet = *current_it;
//     std::shared_ptr<StateNode> current_state = dicretize_grid[*current_it];

//     for (auto it = open_set.begin(); it != open_set.end(); it++) {
//       if (dicretize_grid[(*it)]->f < current_state->f) {
//         current_it = it;
//         current_triplet = *it;
//         current_state = dicretize_grid[*it];
//       }
//     }

//     closed_set.insert(current_triplet);

//     if (IsGoal(*current_state, target_indexes)) {
//       if (current_state->parent == nullptr) {
//         command = MotionPrimitive();
//       } else {
//         std::shared_ptr<StateNode> runner = current_state;
//         while (runner->parent->parent != nullptr) {
//           runner = runner->parent;
//         }
//         command = runner->motion_from_parent;
//       }
//       return true;
//     }

//     open_set.erase(current_it);

//     for (MotionPrimitive motion : motion_primitives_) {

//       std::shared_ptr<StateNode> neighbor = GetNeighbor(*current_state, motion);

//       // Check if inside grid
//       if (!Accessible(*neighbor)) {
//         continue;
//       }

//       std::tuple<int, int, int> neighbor_triplet = GetTriplet(*neighbor);
//       if (closed_set.find(neighbor_triplet) != closed_set.end()) {
//         continue;
//       }
      
//       neighbor->parent = current_state;
//       neighbor->f = neighbor->g + GetHeuristicFromPose(*neighbor, h_local_grid);

//       std::vector<std::tuple<int, int, int>>::iterator it = std::find(open_set.begin(), open_set.end(), neighbor_triplet);
//       if (it != open_set.end()) {
//         if (neighbor->g < dicretize_grid[neighbor_triplet]->g) {
//           dicretize_grid[neighbor_triplet] = neighbor;
//         }
//       } else {
//         open_set.push_back(neighbor_triplet);
//         dicretize_grid[neighbor_triplet] = neighbor;
//       }
//     }
//   }

//   return false;

//   // std::unordered_set<std::tuple<int, int, int>, hash_triplet> close_set;

//   // std::vector<std::shared_ptr<StateNode>> open_set;  // Replace with heap ?
//   // open_set.push_back(start);

//   // int i = 0;

//   // while (!open_set.empty()) {

//   //   std::cout << "Explore node nbr: " << i << "\n";
//   //   std::cout << "Open set size: " << open_set.size() << "\n";

//   //   std::vector<std::shared_ptr<StateNode>>::iterator current_it = open_set.begin();
//   //   std::shared_ptr<StateNode> current = *current_it;

//   //   for (auto it = open_set.begin(); it != open_set.end(); it++) {
//   //     if ((*it)->f < current->f) {
//   //       current_it = it;
//   //       current = *it;
//   //     }
//   //   }

//   //   if (close_set.find(GetTriplet(*current)) != close_set.end()) {
//   //     open_set.erase(current_it);
//   //     continue;
//   //   }

//   //   if (IsGoal(*current, target_indexes)) {
//   //     if (current->parent == nullptr) {
//   //       command = MotionPrimitive();
//   //     } else {
//   //       std::shared_ptr<StateNode> runner = current;
//   //       while (runner->parent->parent != nullptr) {
//   //         runner = runner->parent;
//   //       }
//   //       command = runner->motion_from_parent;
//   //     }
//   //     return true;
//   //   }

//   //   open_set.erase(current_it);

//   //   close_set.insert(GetTriplet(*current));

//   //   for (MotionPrimitive motion : motion_primitives_) {

//   //     std::shared_ptr<StateNode> neighbor = GetNeighbor(*current, motion);

//   //     // Check if inside grid
//   //     if (!Accessible(*neighbor)) {
//   //       continue;
//   //     }
//   //     neighbor->parent = current;

//   //     if (close_set.find(GetTriplet(*neighbor)) != close_set.end()) {
//   //       continue;
//   //     }

//   //     neighbor->f = neighbor->g + GetHeuristicFromPose(*neighbor, h_local_grid);
//   //     open_set.push_back(neighbor);
//   //   }

//   //   ++i;

//   // }

//   // return false;
// }

// float LocalGrid::GetHeuristicFromPose(const StateNode& state, const std::vector<float>& h_local_grid) {
//   int idx_1 = (int) (state.x / 0.03f);
//   int idx_2 = (int) (state.y / 0.03f);
//   return h_local_grid[idx_1 + 128 * idx_2];
// }

// bool LocalGrid::IsGoal(const StateNode& state, std::pair<int, int> target_indexes) {
//   Eigen::Vector2f state_pose(state.x, state.y);
//   Eigen::Vector2f target_pose((target_indexes.first + 0.5f) * 0.03f, (target_indexes.second + 0.5f) * 0.03f);
//   return (state_pose - target_pose).norm() <= min_dist_to_target_;
// }

// std::shared_ptr<StateNode> LocalGrid::GetNeighbor(const StateNode& state, const MotionPrimitive motion) {
//   std::shared_ptr<StateNode> neighbor = std::make_shared<StateNode>();
//   neighbor->x = state.x + motion.x;
//   neighbor->y = state.y + motion.y;
//   if (state.theta + motion.angle > M_PI) {
//     neighbor->theta = state.theta + motion.angle - 2 * M_PI;
//   } else if (state.theta + motion.angle < -M_PI) {
//     neighbor->theta = state.theta + motion.angle + 2 * M_PI;
//   }
//   neighbor->motion_from_parent = motion;
//   neighbor->g = state.g + motion.cost;
//   return neighbor;
// }

// bool LocalGrid::Accessible(const StateNode& state) {

//   int idx_1 = (int) (state.x / 0.03f);
//   int idx_2 = (int) (state.y / 0.03f);

//   if (idx_1 < 0 || idx_2 < 0 || idx_1 >= 128 || idx_2 >= 128) {
//     return false;
//   }

//   if (local_grid_[idx_1 + 128 * idx_2] < min_dist_) {
//     return false;
//   }

//   int dx = std::cos(state.theta);
//   int dy = std::sin(state.theta);
//   for (int i = 0; i < 9; ++i) {
//     float alpha = -0.5f + 0.125f * i;
//     idx_1 = (int) ((state.x + alpha * dx) / 0.03f);
//     idx_2 = (int) ((state.y + alpha * dy) / 0.03f);
//     if (idx_1 < 0 || idx_2 < 0 || idx_1 >= 128 || idx_2 >= 128) {
//       continue;
//     }
//     if (local_grid_[idx_1 + 128 * idx_2] < min_dist_) {
//       return false;
//     }
//   }
//   return true;
// }

// std::tuple<int, int, int> LocalGrid::GetTriplet(const StateNode& state) {
//   int idx_1 = (int) (state.x / 0.06f);
//   int idx_2 = (int) (state.y / 0.06f);
//   int idx_3 = (int) ((state.theta + M_PI) / (M_PI / 8));
//   // int idx_3 = 0;
//   return {idx_1, idx_2, idx_3};
// }

// nav_msgs::msg::OccupancyGrid LocalGrid::LocalGridToOccupancyGrid() {

//   std::pair<int, int> target_indexes(64, 64);

//   // Compute dijkstra heuristic for the A* search
//   std::vector<float> h_local_grid;
//   ComputeHeuristic(target_indexes, h_local_grid);

//   nav_msgs::msg::OccupancyGrid occupancy_grid;

//   float max_h = -std::numeric_limits<float>::max();
//   for (int i = 0; i < (int) h_local_grid.size(); ++i) {
//     if (h_local_grid[i] > max_h) {
//       max_h = h_local_grid[i];
//     }
//   }

//   // occupancy_grid.data.reserve(128 * 128);

//   for (int i = 0; i < 128 * 128; ++i) {
//     if (h_local_grid[i] < 0.25f) {
//       occupancy_grid.data.push_back(100);
//     } else {
//       int value = (int) h_local_grid[i] / max_h * 100;
//       occupancy_grid.data.push_back(value);
//     }
//   }

//   Eigen::Matrix4f local_grid_to_base_link = aut_utils::InverseTransformation(base_link_to_local_grid_);
//   Eigen::Matrix4f base_link_to_x_axis = Eigen::Matrix4f::Identity();
//   base_link_to_x_axis(0, 3) = 1.0f;
//   Eigen::Matrix4f local_grid_to_x_axis = local_grid_to_base_link * base_link_to_x_axis;

//   float dx = local_grid_to_x_axis(0, 3) - local_grid_to_base_link(0, 3);
//   float dy = local_grid_to_x_axis(1, 3) - local_grid_to_base_link(1, 3);
//   float heading = std::atan2(dy, dx);

//   std::shared_ptr<StateNode> start = std::make_shared<StateNode>();
//   start->x = local_grid_to_base_link(0, 3);
//   start->y = local_grid_to_base_link(1, 3);
//   start->theta = heading;
//   start->g = 0;
//   start->f = GetHeuristicFromPose(*start, h_local_grid);

//   std::cout << "X_idx : " << (int) ((start->x) / 0.03f) << ", Y_idx : " << (int) ((start->y) / 0.03f) <<  "\n";

//   dx = std::cos(start->theta);
//   dy = std::sin(start->theta);

//   for (int i = 0; i < 9; ++i) {
//     float alpha = -0.5f + 0.125f * i;

//     int idx_1 = (int) ((start->x + alpha * dx) / 0.03f);
//     int idx_2 = (int) ((start->y + alpha * dy) / 0.03f);

//     std::cout << "X_idx : " << idx_1 << ", Y_idx : " << idx_2 <<  "\n";

//     if (idx_1 < 0 || idx_2 < 0 || idx_1 >= 128 || idx_2 >= 128) {
//       continue;
//     }

//     occupancy_grid.data[idx_1 + 128 * idx_2] = 0;
//   }


//   // float max_h = -std::numeric_limits<float>::max();
//   // for (int i = 0; i < (int) h_local_grid.size(); ++i) {
//   //   if (h_local_grid[i] > max_h) {
//   //     max_h = h_local_grid[i];
//   //   }
//   // }

//   // occupancy_grid.data.reserve(128 * 128);

//   // for (int i = 0; i < 128; ++i) {
//   //   for (int j = 0; j < 128; ++j) {
//   //     if (local_grid_[i + 128 * j] < 0.25f) {
//   //       occupancy_grid.data.push_back(100);
//   //     } else {
//   //       occupancy_grid.data.push_back(0);
//   //     }
//   //   }
//   // }

//   // for (int i = 0; i < 128 * 128; ++i) {
//   //   if (local_grid_[i] < 0.25f) {
//   //     occupancy_grid.data.push_back(100);
//   //   } else {
//   //     occupancy_grid.data.push_back(0);
//   //   }
//   // }

//   occupancy_grid.header.frame_id = std::string("base_link");

//   occupancy_grid.info.resolution = 0.03;
//   occupancy_grid.info.width = 128;
//   occupancy_grid.info.height = 128;

//   geometry_msgs::msg::Pose base_link_to_local_grid_pose = aut_utils::MatrixToPose(base_link_to_local_grid_);

//   occupancy_grid.info.origin.position.x = base_link_to_local_grid_pose.position.x;
//   occupancy_grid.info.origin.position.y = base_link_to_local_grid_pose.position.y;
//   occupancy_grid.info.origin.position.z = base_link_to_local_grid_pose.position.z;

//   occupancy_grid.info.origin.orientation.x = base_link_to_local_grid_pose.orientation.x;
//   occupancy_grid.info.origin.orientation.y = base_link_to_local_grid_pose.orientation.y;
//   occupancy_grid.info.origin.orientation.z = base_link_to_local_grid_pose.orientation.z;
//   occupancy_grid.info.origin.orientation.w = base_link_to_local_grid_pose.orientation.w;

//   return occupancy_grid;
// }

// }  // namespace aut_local_planner

// #include "aut_local_planner/local_grid.h"

// #include <vector>
// #include <utility>
// #include <unordered_map>
// #include <algorithm>
// #include <cmath>

// #include <Eigen/Core>

// #include "aut_utils/utils.h"

// namespace aut_local_planner {

// LocalGrid::LocalGrid(float min_dist, float max_dist) {
//   min_dist_ = min_dist;
//   max_dist_ = max_dist;
//   obstacle_coeff_ = 3.0f;
// }

// void LocalGrid::AddLocalGrid(std::vector<float> local_grid, Eigen::Matrix4f map_to_base_link, Eigen::Matrix4f base_link_to_local_grid) {
//   local_grid_ = local_grid;
//   map_to_base_link_ = map_to_base_link;
//   base_link_to_local_grid_ = base_link_to_local_grid;
// }

// bool LocalGrid::IsInside(Eigen::Vector3f map_to_position_translation) {
//   // Get pose of position inside the local grid
//   Eigen::Matrix4f map_to_local_grid = map_to_base_link_ * base_link_to_local_grid_;
//   Eigen::Matrix4f local_grid_to_map = aut_utils::InverseTransformation(map_to_local_grid);

//   Eigen::Matrix4f map_to_position = Eigen::Matrix4f::Identity();
//   map_to_position.block<3, 1>(0, 3) = map_to_position_translation;

//   Eigen::Matrix4f local_grid_to_position = local_grid_to_map * map_to_position;

//   // Transform the pose in local grid coordinate
//   int idx_1 = (int) (local_grid_to_position(0, 3) / 0.03f);
//   int idx_2 = (int) (local_grid_to_position(1, 3) / 0.03f);

//   return idx_1 >= 0 && idx_2 >= 0 && idx_1 < 128 && idx_2 < 128;
// }

// bool LocalGrid::FindPath(Eigen::Vector3f map_to_position_translation, Eigen::Vector2f& direction) {

//   // Get pose of position inside the local grid
//   Eigen::Matrix4f map_to_local_grid = map_to_base_link_ * base_link_to_local_grid_;
//   Eigen::Matrix4f local_grid_to_map = aut_utils::InverseTransformation(map_to_local_grid);

//   Eigen::Matrix4f map_to_position = Eigen::Matrix4f::Identity();
//   map_to_position.block<3, 1>(0, 3) = map_to_position_translation;

//   Eigen::Matrix4f local_grid_to_position = local_grid_to_map * map_to_position;

//   // Transform the pose in local grid coordinate
//   std::pair<int, int> goal((int) (local_grid_to_position(0, 3) / 0.03f), (int) (local_grid_to_position(1, 3) / 0.03f));

//   Eigen::Matrix4f local_grid_to_base_link = aut_utils::InverseTransformation(base_link_to_local_grid_);

//   std::pair<int, int> start((int) (local_grid_to_base_link(0, 3) / 0.03f), (int) (local_grid_to_base_link(1, 3) / 0.03f));

//   if (!IsInGrid(goal) || !IsInGrid(start)) {
//     return false;
//   }

//   std::vector<std::pair<int, int>> path;
//   bool found_path = AStar(start, goal, path);

//   if (!found_path) {
//     return false;
//   }

//   std::pair<int, int> target = path[std::min(8, (int) path.size())];
//   Eigen::Matrix4f local_grid_to_target = Eigen::Matrix4f::Identity();
//   local_grid_to_target(0, 3) = (target.first + 0.5) * 0.03;
//   local_grid_to_target(1, 3) = (target.second + 0.5) * 0.03;
//   Eigen::Matrix4f base_link_to_target = base_link_to_local_grid_ * local_grid_to_target;
//   direction(0) = base_link_to_target(0, 3);
//   direction(1) = base_link_to_target(1, 3);
//   return true;
// }

// bool LocalGrid::AStar(std::pair<int, int> start, std::pair<int, int> goal, std::vector<std::pair<int, int>> &path) {

//   std::unordered_map<std::pair<int, int>, std::pair<int, int>, hash_pair> came_from;

//   std::unordered_map<std::pair<int, int>, float, hash_pair> f_score;
//   f_score[start] = GetEuclideanDist(start, goal);

//   std::unordered_map<std::pair<int, int>, float, hash_pair> g_score;
//   g_score[start] = 0;

//   std::vector<std::pair<int, int>> open_set;
//   open_set.push_back(start);

//   while (!open_set.empty()) {
//     std::vector<std::pair<int, int>>::iterator current_it = open_set.begin();
//     std::pair<int, int> current = *current_it;

//     for (auto it = open_set.begin(); it != open_set.end(); it++) {
//       if (f_score[*it] < f_score[current]) {
//         current_it = it;
//         current = *it;
//       }
//     }

//     // if (current == goal) { // Is it a good idea ? Or need to do like dijkstra and find all to find best path

//     //   while (came_from.find(current) != came_from.end()) {
//     //     path.push_back(current);
//     //     current = came_from[current];
//     //   }

//     //   path.push_back(start);

//     //   std::reverse(path.begin(), path.end());

//     //   return true;
//     // }

//     open_set.erase(current_it);

//     std::vector<std::pair<int, int>> neighbors = GetNeighbors(current);

//     for (std::pair<int, int> neighbor: neighbors) {

//       float tentative_g_score = g_score[current] + GetEuclideanDist(current, neighbor) * (1 + obstacle_coeff_ * GetWeight(current)); // Change weight

//       float neighbor_g_score = g_score.find(neighbor) == g_score.end() ? std::numeric_limits<float>::max() : g_score[neighbor];

//       if (tentative_g_score < neighbor_g_score) { // Add it because we have found better

//         came_from[neighbor] = current;
//         g_score[neighbor] = tentative_g_score;
//         f_score[neighbor] = tentative_g_score + GetEuclideanDist(neighbor, goal);

//         std::vector<std::pair<int, int>>::iterator it = std::find(open_set.begin(), open_set.end(), neighbor);
//         if (it == open_set.end()) {
//           open_set.push_back(neighbor);
//         }
//       }
//     }
//   }

//   if (came_from.find(goal) != came_from.end()) { // Is it a good idea ? Or need to do like dijkstra and find all to find best path

//     std::pair<int, int> current = goal;

//     while (came_from.find(current) != came_from.end()) {
//       path.push_back(current);
//       current = came_from[current];
//     }

//     path.push_back(start);

//     std::reverse(path.begin(), path.end());

//     return true;
//   }

//   return false;
// }

// std::vector<std::pair<int, int>> LocalGrid::GetNeighbors(std::pair<int, int> p) {
//   std::vector<std::pair<int, int>> neighbors;
//   for (int i = -1; i < 2; i++) {
//     for (int j = -1; j < 2; j++) {
//       if (i == 0 && j == 0) {
//         continue;
//       }
//       std::pair<int, int> neighbor(p.first + i, p.second + j);
//       if (!IsFree(neighbor)) {
//         continue;
//       }
//       neighbors.push_back(neighbor);
//     }
//   }
//   return neighbors;
// }

// bool LocalGrid::IsInGrid(std::pair<int, int> p) {
//   return p.first >= 0 && p.first < 128 && p.second >= 0 && p.second < 128;
// }

// bool LocalGrid::IsFree(std::pair<int, int> p) {
//   if (!IsInGrid(p)) {
//     return false;
//   }
//   return local_grid_[p.first + 128 * p.second] >= min_dist_;
// }

// float LocalGrid::GetEuclideanDist(std::pair<int, int> p1, std::pair<int, int> p2) {
//   return 0.03 * std::sqrt((p1.first - p2.first) * (p1.first - p2.first) + (p1.second - p2.second) * (p1.second - p2.second));
// }

// float LocalGrid::GetWeight(std::pair<int, int> p) {
//   float obstacle_dist = local_grid_[p.first + 128 * p.second];
//   if (obstacle_dist <= min_dist_) {
//     return 1.0f;
//   }
//   if (obstacle_dist > max_dist_) {
//     return 0.0f;
//   }
//   float t = (obstacle_dist - min_dist_) / (max_dist_ - min_dist_); 

//   return 1 - (t * t * (3 - 2 * t));
// }

// }  // namespace aut_local_planner