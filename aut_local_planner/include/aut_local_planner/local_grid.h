#ifndef AUT_LOCAL_PLANNER_LOCAL_GRID_H_
#define AUT_LOCAL_PLANNER_LOCAL_GRID_H_

#include <vector>
#include <utility>

#include <Eigen/Core>

namespace aut_local_planner {

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

class LocalGrid {

 public:
  explicit LocalGrid(float min_dist, float max_dist);

  void AddLocalGrid(std::vector<float> local_grid, Eigen::Matrix4f map_to_base_link, Eigen::Matrix4f base_link_to_local_grid);

  bool IsInside(Eigen::Vector3f map_to_position_translation);

  bool FindPath(Eigen::Vector3f map_to_position_translation, Eigen::Vector2f& direction);

  bool AStar(std::pair<int, int> start, std::pair<int, int> goal, std::vector<std::pair<int, int>> &path);

  std::vector<std::pair<int, int>> GetNeighbors(std::pair<int, int> p);
  bool IsInGrid(std::pair<int, int> p);
  bool IsFree(std::pair<int, int> p);
  float GetEuclideanDist(std::pair<int, int> p1, std::pair<int, int> p2);
  float GetWeight(std::pair<int, int> p);
 private:

  // Parameters
  float min_dist_;
  float max_dist_;
  float obstacle_coeff_;

  // State
  std::vector<float> local_grid_;
  Eigen::Matrix4f map_to_base_link_;
  Eigen::Matrix4f base_link_to_local_grid_;
};

}  // namespace aut_local_planner

#endif  // AUT_LOCAL_PLANNER_LOCAL_GRID_H_