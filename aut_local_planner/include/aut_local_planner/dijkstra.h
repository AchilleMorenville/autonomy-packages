#ifndef AUT_LOCAL_PLANNER_DIJKSTRA_H_
#define AUT_LOCAL_PLANNER_DIJKSTRA_H_

#include <vector>

namespace aut_local_planner {

class Dijkstra {
 public:
  Dijkstra(int x_dim, int y_dim, float cell_size, float min_obstacle_dist);
  void Compute(float x, float y, const std::vector<float>& obstacle_grid, std::vector<float>& dijkstra_grid);
 private:
 
  bool Accessible(const std::pair<int, int>& p);
  bool InsideGrid(const std::pair<int, int>& p);
  bool IsFree(const std::pair<int, int>& p);
  int GetIndex(const std::pair<int, int>& p);
  void FindNeighbors(
      const std::pair<std::pair<int, int>, float>& state,
      std::vector<std::pair<std::pair<int, int>, float>>& neighbors);

  int x_dim_;
  int y_dim_;
  float cell_size_;
  float min_obstacle_dist_;
  std::vector<float> obstacle_grid_;
}; 

}

#endif  // AUT_LOCAL_PLANNER_DIJKSTRA_H_