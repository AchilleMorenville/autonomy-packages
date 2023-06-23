#include "aut_local_planner/grid.h"

#include <array>
#include <vector>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <cmath>

#include <Eigen/Core>

#include "aut_utils/utils.h"

namespace aut_local_planner {

Grid::Grid() {
  initialized_ = false;
  size = 256;
  min_dist_ = 0.25f;
  max_dist_ = 0.7f;
  grid_to_center_position_ = Eigen::Matrix4f::Identity();
  grid_to_center_position(0, 3) = (size_ / 2) / 0.03;
  grid_to_center_position(1, 3) = (size_ / 2) / 0.03;
  grid_to_center_position(1, 0) = 1.0f;
  grid_to_center_position(0, 1) = -1.0f;

  grid_.fill(1.0);
  grid_seen_.fill(false);
}

void Grid::Initialize(Vector3f center_position) {
  center_position_ = center_position;
  map_to_center_position_ = Eigen::Matrix4f::Identity();
  map_to_center_position_.block<3, 1>(0, 3) = center_position;
  initialized_ = true;
}

void Grid::AddLocalGrid(std::vector<float> local_grid, Eigen::Matrix4f map_to_base_link, Eigen::Matrix4f base_link_to_local_grid) {

  Eigen::Matrix4f grid_to_map = grid_to_center_position_ * aut_utils::InverseTransformation(map_to_center_position_);
  Eigen::Matrix4f grid_to_local_grid = grid_to_map * (map_to_base_link * base_link_to_local_grid);

  for (int i = 0; i < (int) local_grid.size(); ++i) {
    float x = (i / 128 + 0.5f) * 0.03f;
    float y = (i % 128 + 0.5f) * 0.03f;

    Eigen::Matrix4f local_grid_to_point = Eigen::Matrix4f::Identity();
    local_grid_to_point(0, 3) = x;
    local_grid_to_point(1, 3) = y;

    Eigen::Matrix4f grid_to_point = grid_to_local_grid * local_grid_to_point;

    int idx_1 = (int) (grid_to_point(0, 3) / 0.03);
    int idx_2 = (int) (grid_to_point(1, 3) / 0.03);

    if (idx_1 < 0 || idx_2 < 0 || idx_1 >= size_ || idx_2 >= size_) {
      continue;
    }

    grid_[idx_1 + size_ * idx_2] = local_grid[i];
    grid_seen_[idx_1 + size_ * idx_2] = true;
  }
}

bool Grid::IsFreeAndSeen(Vector3f map_to_position) {
  Eigen::Matrix4f map_to_position_matrix = Eigen::Matrix4f::Identity();
  map_to_position_matrix.block<3, 1>(0, 3) = map_to_position;
  Eigen::Matrix4f grid_to_map = grid_to_center_position_ * aut_utils::InverseTransformation(map_to_center_position_);

  Eigen::Matrix4f grid_to_position = grid_to_map * map_to_position_matrix;
  int idx_1 = (int) (grid_to_position(0, 3) / 0.03);
  int idx_2 = (int) (grid_to_position(1, 3) / 0.03);

  if (idx_1 < 0 || idx_2 < 0 || idx_1 >= size_ || idx_2 >= size_) {
    return false;
  }

  return grid_[idx_1 + idx_2 * size_] > min_dist_ && grid_seen_[idx_1 + idx_2 * size_]
}

void Grid::Move(Vector3f new_center_position) {
  Eigen::Vector3f delta = new_center_position - center_position_;

  int delta_idx_1 = - delta(1);
  int delta_idx_2 = delta(0);

  std::array<float, 65536> old_grid = grid_;
  std::array<bool, 65536> old_grid_seen = grid_seen_;

  for (int i = 0; i < size_ * size_; ++i) {

    int idx_1 = i / size_;
    int idx_2 = i % size_;

    int new_idx_1 = idx_1 + delta_idx_1;
    int new_idx_2 = idx_2 + delta_idx_2;

    if (new_idx_1 < 0 || new_idx_2 < 0 || new_idx_1 >= size_ || new_idx_2 >= size_) {
      grid_[i] = 1.0f;
      grid_seen_[i] = false;
    } else {
      grid_[i] = old_grid_[new_idx_1 + new_idx_2 * size_];
      grid_seen_[i] = old_grid_seen_[new_idx_1 + new_idx_2 * size_];
    }
  }

  center_position_ = new_center_position;
}

bool Grid::GetDirection(Eigen::Matrix4f map_to_base_link, Eigen::Vector2f& direction) {
  Eigen::Matrix4f grid_to_map = grid_to_center_position_ * aut_utils::InverseTransformation(map_to_center_position_);
  Eigen::Matrix4f grid_to_base_link = grid_to_map * map_to_base_link;

  int idx_1 = (int) (grid_to_base_link(0, 3) / 0.03);
  int idx_2 = (int) (grid_to_base_link(1, 3) / 0.03);

  if (idx_1 < 0 || idx_2 < 0 || idx_1 >= size_ || idx_2 >= size_) {
    return false;
  }

  std::vector<std::pair<int, int>> path;
  std::pair<int, int> goal(size_ / 2 + size_ / 2 * size_);
  std::pair<int, int> start(idx_1 + idx_2 * size_);
  bool found_path = AStar(start, goal, path);

  if (!found_path) {
    return false;
  }

  std::pair<int, int> target = path[std::min(10, path.size())];

  Eigen::Matrix4f grid_to_target = Eigen::Matrix4f::Identity();
  grid_to_target(0, 3) = (target.first + 0.5) * 0.03;
  grid_to_target(1, 3) = (target.second + 0.5) * 0.03;

  Eigen::Matrix4f base_link_to_target = aut_utils::InverseTransformation(grid_to_base_link) * grid_to_target;
  direction(0) = base_link_to_target(0, 3);
  direction(1) = base_link_to_target(1, 3);
  return true;
}

bool Grid::AStar(std::pair<int, int> start, std::pair<int, int> goal, std::vector<std::pair<int, int>> &path) {

  std::unordered_map<std::pair<int, int>, std::pair<int, int>, hash_pair> came_from;

  std::unordered_map<std::pair<int, int>, float, hash_pair> f_score;
  f_score[start] = getEuclideanDist(start, goal);

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

      float tentative_g_score = g_score[current] + GetEuclideanDist(current, neighbor) * (1 + obstacle_coeff * GetWeight(current)); // Change weight

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

std::vector<std::pair<int, int>> Grid::GetNeighbors(std::pair<int, int> p) {

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

bool Grid::IsInGrid(std::pair<int, int> p) {
  return p.first >= 0 && p.first < 256 && p.second >= 0 && p.second < 256;
}

bool Grid::IsFree(std::pair<int, int> p) {
  if (!isInGrid(p)) {
    return false;
  }
  return grid_[p.first + 256 * p.second] >= min_dist_;
}

float Grid::GetEuclideanDist(std::pair<int, int> p1, std::pair<int, int> p2) {
  return 0.03 * std::sqrt((p1.first - p2.first) * (p1.first - p2.first) + (p1.second - p2.second) * (p1.second - p2.second));
}

float Grid::GetWeight(std::pair<int, int> p) {
  float obstacle_dist = grid_[p.first + 256 * p.second];

  if (obstacle_dist <= min_dist_) {
    return 1.0f;
  }

  if (obstacle_dist > max_dist_) { // Max dist
    return 0.0f;
  }

  float t = (obstacle_dist - min_dist_) / (max_dist_ - min_dist_); 

  return 1 - (t * t * (3 - 2 * t));
}

void Grid::Reset() {
  initialized_ = false;
  size = 256;
  grid_to_center_position_ = Eigen::Matrix4f::Identity();
  grid_to_center_position(0, 3) = (size_ / 2) / 0.03;
  grid_to_center_position(1, 3) = (size_ / 2) / 0.03;
  grid_to_center_position(1, 0) = 1.0f;
  grid_to_center_position(0, 1) = -1.0f;

  grid_.fill(1.0);
  grid_seen_.fill(false);
}

// Grid::Grid() {
//   initialized_ = false;
//   size = 250;
//   idx_center_position_ = -1;

//   grid_to_center_position_ = Eigen::Matrix4f::Identity();
//   grid_to_center_position(0, 3) = (size_ / 2) / 0.03;
//   grid_to_center_position(1, 3) = (size_ / 2) / 0.03;
//   grid_to_center_position(1, 0) = 1.0f;
//   grid_to_center_position(0, 1) = -1.0f;

//   grid_.fill(1.0f);

// }

// void Grid::SetPath(std::vector<Eigen::Vector3f> p) {
//   path_ = p;

//   if (initialized_) {
//     Reset()
//   }

//   center_position_ = path_[0];
//   idx_center_position_ = 0;
//   initialized_ = true;
// }

// void Grid::AddLocalGrid(std::vector<float> local_grid, Eigen::Matrix4f map_to_base_link, Eigen::Matrix4f base_link_to_local_grid) {

//   Eigen::Matrix4f map_to_center_position = Eigen::Matrix4f::Identity();
//   map_to_center_position.block<3, 1>(0, 3) = grid_position_;
//   Eigen::Matrix4f grid_to_map = grid_to_center_position_ * aut_utils::InverseTransformation(map_to_center_position);

//   if ((map_to_base_link.block<3, 1>(0, 3) - center_position_).norm() <= 1.8) {
//     MoveToNextCenter();
//   }


//   Eigen::Matrix4f grid_to_local_grid = grid_to_map * (map_to_base_link * base_link_to_local_grid);

//   for (int i = 0; i < (int) local_grid.size(); ++i) {
//     float x = (i / 128 + 0.5f) * 0.03f;
//     float y = (i % 128 + 0.5f) * 0.03f;

//     Eigen::Matrix4f local_grid_to_point = Eigen::Matrix4f::Identity();
//     local_grid_to_point(0, 3) = x;
//     local_grid_to_point(1, 3) = y;

//     Eigen::Matrix4f grid_to_point = grid_to_local_grid * local_grid_to_point;

//     int idx_1 = (int) (grid_to_point(0, 3) / 0.03);
//     int idx_2 = (int) (grid_to_point(1, 3) / 0.03);

//     if (idx_1 < 0 || idx_2 < 0 || idx_1 >= size_ || idx_2 >= size_) {
//       continue;
//     }

//     grid_[idx_1 + size_ * idx_2] = local_grid[i];
//   }
// }

// void Grid::MoveToNextCenter() {

//   if (idx_center_position_ >= (int) path_.size() - 1) {
//     return;
//   }

//   Eigen::Vector3f delta = path_[idx_center_position_ + 1] - center_position_;

//   int delta_idx_1 = - delta(1);
//   int delta_idx_2 = delta(0);

//   std::array<float, 62500> old_grid = grid_;

//   for (int i = 0; i < size_ * size_; ++i) {

//     int idx_1 = i / size_;
//     int idx_2 = i % size_;

//     int new_idx_1 = idx_1 + delta_idx_1;
//     int new_idx_2 = idx_2 + delta_idx_2;

//     if (new_idx_1 < 0 || new_idx_2 < 0 || new_idx_1 >= size_ || new_idx_2 >= size_) {
//       grid_[i] = 1.0f;
//     } else {
//       grid_[i] = old_grid_[new_idx_1 + new_idx_2 * size_];
//     }
//   }

//   center_position_ = path_[idx_center_position_ + 1];
//   idx_center_position_++;
// }

// void Grid::Reset() {
//   grid_.fill(1.0f);
//   idx_center_position_ = -1;
//   center_position_ = Eigen::Vector3f(0, 0, 0);
// }



}  // namespace aut_local_planner