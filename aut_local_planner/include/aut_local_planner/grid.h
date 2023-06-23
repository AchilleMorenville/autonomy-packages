#ifndef AUT_LOCAL_PLANNER_GRID_H_
#define AUT_LOCAL_PLANNER_GRID_H_

#include <unordered_map>
#include <vector>
#include <array>
#include <string>

#include <Eigen/Core>

namespace aut_local_planner {

// class Grid {

//  public:
//   explicit Grid();

//   void SetPath(std::vector<Eigen::Vector3f> p);

//   void AddLocalGrid(std::vector<float> local_grid, Eigen::Matrix4f map_to_base_link, Eigen::Matrix4f base_link_to_local_grid);

//  private:

//   void MoveToNextCenter();
//   void Reset();

//   // Parameters
//   int size_;

//   // State
//   std::vector<Eigen::Vector3f> path_;

//   bool initialized_;

//   std::array<float, 62500> grid_; // Grid of 250 by 250

//   Eigen::Vector3f grid_position_;

//   Eigen::Matrix4f grid_to_center_position_;

//   int idx_center_position_;

// };

struct hash_pair {
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2>& p) const
  {
    std::size_t hash1 = std::hash<T1>{}(p.first);
    std::size_t hash2 = std::hash<T2>{}(p.second);
    std::size_t r = 0;
    r ^= hash1 + 0x9e3779b9 + (r << 6) + (r >> 2);
    r ^= hash2 + 0x9e3779b9 + (r << 6) + (r >> 2);
    return r;
  }
};

class Grid {

 public:
  explicit Grid();

  void Initialize(Vector3f center_position);
  void Reset();

  void AddLocalGrid(std::vector<float> local_grid, Eigen::Matrix4f map_to_base_link, Eigen::Matrix4f base_link_to_local_grid);
  bool IsFreeAndSeen(Vector3f map_to_position);

  void Move(Vector3f new_center_position);

  bool GetDirection(Eigen::Matrix4f map_to_base_link, Eigen::Vector2f& direction);

 private:

  bool AStar(std::pair<int, int> start, std::pair<int, int> goal, std::vector<std::pair<int, int>> &path);
  std::vector<std::pair<int, int>> GetNeighbors(std::pair<int, int> p);
  bool IsInGrid(std::pair<int, int> p);
  bool IsFree(std::pair<int, int> p);
  float GetEuclideanDist(std::pair<int, int> p1, std::pair<int, int> p2);
  float GetWeight(std::pair<int, int> p);

  // Parameters
  int size_;
  float min_dist_;
  float max_dist_;

  // State
  bool initialized_;

  std::array<float, 65536> grid_; // Grid of 256 by 256
  std::array<bool, 65536> grid_seen_;

  Eigen::Vector3f center_position_;
  Eigen::Matrix4f map_to_center_position_;
  Eigen::Matrix4f grid_to_center_position_;
};

}  // namespace aut_local_planner

#endif  // AUT_LOCAL_PLANNER_GRID_H_