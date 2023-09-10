#include "aut_local_planner/dijkstra.h"

#include <vector>
#include <unordered_map>
#include <utility>
#include <cmath>
// #include <iostream>
// #include <string>

namespace aut_local_planner {

Dijkstra::Dijkstra(int x_dim, int y_dim, float cell_size, 
                   float min_obstacle_dist) 
    : x_dim_(x_dim), 
      y_dim_(y_dim), 
      cell_size_(cell_size), 
      min_obstacle_dist_(min_obstacle_dist) {}
  
void Dijkstra::Compute(float x, float y, 
                       const std::vector<float>& obstacle_grid, 
                       std::vector<float>& dijkstra_grid) {
  obstacle_grid_ = obstacle_grid;                 
  dijkstra_grid = std::vector<float>(x_dim_ * y_dim_, -1.0f);
  std::pair<int, int> start{static_cast<int>(x / cell_size_), 
                            static_cast<int>(y / cell_size_)};
  if (!Accessible(start)) { return; }
  
  std::unordered_map<int, std::pair<std::pair<int, int>, float>> open_set;
  open_set[GetIndex(start)] = {start, 0.0f};

  while (!open_set.empty()) {
    auto best_it = open_set.begin();
    for (auto it = open_set.begin(); it != open_set.end(); ++it) {
      if (it->second.second < best_it->second.second) {
        best_it = it;
      }
    }
    dijkstra_grid[best_it->first] = best_it->second.second;
    std::pair<std::pair<int, int>, float> best = best_it->second;
    open_set.erase(best_it);
    
    std::vector<std::pair<std::pair<int, int>, float>> neighbors;
    FindNeighbors(best, neighbors);
    for (auto& neighbor : neighbors) {
      if (dijkstra_grid[GetIndex(neighbor.first)] >= 0.0f) { continue; }
      auto it = open_set.find(GetIndex(neighbor.first));
      if (it != open_set.end()) {
        it->second.second = std::min(it->second.second, neighbor.second);
      } else {
        open_set[GetIndex(neighbor.first)] = neighbor;
      }
    }
  }
  for (int i = 0; i < (int) dijkstra_grid.size(); ++i) {
    if (dijkstra_grid[i] > 0.0f) {
      dijkstra_grid[i] *= cell_size_;
    }
  }
}

bool Dijkstra::Accessible(const std::pair<int, int>& p) {
  return InsideGrid(p) && IsFree(p);
}

bool Dijkstra::InsideGrid(const std::pair<int, int>& p) {
  return p.first >= 0 && p.first < x_dim_ && p.second >= 0 && p.second < y_dim_;
}

bool Dijkstra::IsFree(const std::pair<int, int>& p) {
  return obstacle_grid_[GetIndex(p)] >= min_obstacle_dist_;
}

int Dijkstra::GetIndex(const std::pair<int, int>& p) {
  return p.first + p.second * x_dim_;
}

void Dijkstra::FindNeighbors(
    const std::pair<std::pair<int, int>, float>& state,
    std::vector<std::pair<std::pair<int, int>, float>>& neighbors) {
  for (int i = -1; i < 2; ++i) {
    for (int j = -1; j < 2; ++j) {
      if (i == 0 && j == 0) { continue; }
      std::pair<int, int> neighbor{state.first.first + i, 
                                   state.first.second + j};
      if (!Accessible(neighbor)) { continue; }
      neighbors.push_back({neighbor, state.second + std::sqrt(i*i + j*j)});
    }
  }
}

}  // namespace aut_local_planner

// void PrintResult(const std::vector<float>& dijkstra_grid, int x_dim, int y_dim) {
//   for (int i = 0; i < x_dim; ++i) {
//     for (int j = 0; j < y_dim; ++j) {
//       std::cout << dijkstra_grid[i + x_dim * j] << " ";
//     }
//     std::cout << "\n";
//   }
//   std::cout << "-----------\n";
// }

// int main(int argc, char * argv[]) {

//   aut_local_planner::Dijkstra d(3, 3, 0.3f, 0.25f);

//   std::vector<float> dijkstra_grid;
//   std::vector<float> test1 { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
//   d.Compute(0.1f, 0.1f, test1, dijkstra_grid);
//   PrintResult(dijkstra_grid, 3, 3);

//   std::vector<float> test2 { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
//   d.Compute(0.45f, 0.45f, test2, dijkstra_grid);
//   PrintResult(dijkstra_grid, 3, 3);

//   std::vector<float> test3 { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
//   d.Compute(0.1f, 0.45f, test3, dijkstra_grid);
//   PrintResult(dijkstra_grid, 3, 3);

//   std::vector<float> test4 { 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 };
//   d.Compute(0.1f, 0.1f, test4, dijkstra_grid);
//   PrintResult(dijkstra_grid, 3, 3);

//   std::vector<float> test5 { 1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0 };
//   d.Compute(0.1f, 0.1f, test5, dijkstra_grid);
//   PrintResult(dijkstra_grid, 3, 3);

//   std::vector<float> test6 { 1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0 };
//   d.Compute(1.0f, 1.0f, test6, dijkstra_grid);
//   PrintResult(dijkstra_grid, 3, 3);

// }