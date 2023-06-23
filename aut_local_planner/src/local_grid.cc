#include "aut_local_planner/local_grid.h"

#include <vector>
#include <utility>
#include <unordered_map>
#include <algorithm>
#include <cmath>

#include <Eigen/Core>

#include "aut_utils/utils.h"

namespace aut_local_planner {

LocalGrid::LocalGrid(float min_dist, float max_dist) {
  min_dist_ = min_dist;
  max_dist_ = max_dist;
  obstacle_coeff_ = 3.0f;
}

void LocalGrid::AddLocalGrid(std::vector<float> local_grid, Eigen::Matrix4f map_to_base_link, Eigen::Matrix4f base_link_to_local_grid) {
  local_grid_ = local_grid;
  map_to_base_link_ = map_to_base_link;
  base_link_to_local_grid_ = base_link_to_local_grid;
}

bool LocalGrid::IsInside(Eigen::Vector3f map_to_position_translation) {
  // Get pose of position inside the local grid
  Eigen::Matrix4f map_to_local_grid = map_to_base_link_ * base_link_to_local_grid_;
  Eigen::Matrix4f local_grid_to_map = aut_utils::InverseTransformation(map_to_local_grid);

  Eigen::Matrix4f map_to_position = Eigen::Matrix4f::Identity();
  map_to_position.block<3, 1>(0, 3) = map_to_position_translation;

  Eigen::Matrix4f local_grid_to_position = local_grid_to_map * map_to_position;

  // Transform the pose in local grid coordinate
  int idx_1 = (int) (local_grid_to_position(0, 3) / 0.03f);
  int idx_2 = (int) (local_grid_to_position(1, 3) / 0.03f);

  return idx_1 >= 0 && idx_2 >= 0 && idx_1 < 128 && idx_2 < 128;
}

bool LocalGrid::FindPath(Eigen::Vector3f map_to_position_translation, Eigen::Vector2f& direction) {

  // Get pose of position inside the local grid
  Eigen::Matrix4f map_to_local_grid = map_to_base_link_ * base_link_to_local_grid_;
  Eigen::Matrix4f local_grid_to_map = aut_utils::InverseTransformation(map_to_local_grid);

  Eigen::Matrix4f map_to_position = Eigen::Matrix4f::Identity();
  map_to_position.block<3, 1>(0, 3) = map_to_position_translation;

  Eigen::Matrix4f local_grid_to_position = local_grid_to_map * map_to_position;

  // Transform the pose in local grid coordinate
  std::pair<int, int> goal((int) (local_grid_to_position(0, 3) / 0.03f), (int) (local_grid_to_position(1, 3) / 0.03f));

  Eigen::Matrix4f local_grid_to_base_link = aut_utils::InverseTransformation(base_link_to_local_grid_);

  std::pair<int, int> start((int) (local_grid_to_base_link(0, 3) / 0.03f), (int) (local_grid_to_base_link(1, 3) / 0.03f));

  if (!IsInGrid(goal) || !IsInGrid(start)) {
    return false;
  }

  std::vector<std::pair<int, int>> path;
  bool found_path = AStar(start, goal, path);

  if (!found_path) {
    return false;
  }

  std::pair<int, int> target = path[std::min(8, (int) path.size())];
  Eigen::Matrix4f local_grid_to_target = Eigen::Matrix4f::Identity();
  local_grid_to_target(0, 3) = (target.first + 0.5) * 0.03;
  local_grid_to_target(1, 3) = (target.second + 0.5) * 0.03;
  Eigen::Matrix4f base_link_to_target = base_link_to_local_grid_ * local_grid_to_target;
  direction(0) = base_link_to_target(0, 3);
  direction(1) = base_link_to_target(1, 3);
  return true;
}

bool LocalGrid::AStar(std::pair<int, int> start, std::pair<int, int> goal, std::vector<std::pair<int, int>> &path) {

  std::unordered_map<std::pair<int, int>, std::pair<int, int>, hash_pair> came_from;

  std::unordered_map<std::pair<int, int>, float, hash_pair> f_score;
  f_score[start] = GetEuclideanDist(start, goal);

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
        f_score[neighbor] = tentative_g_score + GetEuclideanDist(neighbor, goal);

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
  return 0.03 * std::sqrt((p1.first - p2.first) * (p1.first - p2.first) + (p1.second - p2.second) * (p1.second - p2.second));
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

}  // namespace aut_local_planner