#ifndef AUT_LOCAL_PLANNER_LOCAL_GRID_H_
#define AUT_LOCAL_PLANNER_LOCAL_GRID_H_

#include <vector>
#include <utility>
#include <queue>

#include <Eigen/Core>

#include "aut_local_planner/utils.h"

namespace aut_local_planner {

class LocalGrid {

 public:

  struct Pose {
    float x;
    float y;
    float theta;

    Pose();
    Pose(float x, float y, float theta);
    Pose(const Pose& p);
    Pose& operator=(Pose p);
    ~Pose();
  };

  explicit LocalGrid();
  void SetLocalGrid(const std::vector<float>& local_grid, const Eigen::Matrix4f local_grid_to_base_link, const Eigen::Matrix4f map_to_base_link);
  Motion GetMotion(const Eigen::Matrix4f map_to_goal);

 private:

  struct State {
    Pose pose;
    int parent_idx_3D;
    Motion motion_from_parent;
    float g;
    float h;

    State();
    State(Pose pose, int parent_idx_3D, Motion motion_from_parent, float g, float h);
    State(const State& s);
    State& operator=(State s);
    ~State();
  };

  struct StateComparator {
    bool operator()(const State& a, const State& b) const {
      return (a.g + a.h) > (b.g + b.h);
    }
  };

  typedef std::priority_queue<State, 
                              std::vector<State>, 
                              StateComparator> HybridPriorityQueue;

  int Get3DIndex(Pose pose);
  Pose GetPoseFromMap(Eigen::Matrix4f map_to_p);

  void PrecomputeHeuristics(const Pose goal);
  void PrecomputeObstacleHeuristic(const Pose goal);
  int GetIndex2D(const Pose pose);
  std::vector<std::pair<int, float>> GetNeighbor2D(const std::pair<int, float> current);
  float GetHeuristic(const Pose pose);

  // Parmeters
  int x_size_;
  int y_size_;
  int theta_size_;
  float cell_size_;
  float min_obstacle_dist_;

  std::vector<float> local_grid_;
  std::vector<float> obstacle_heuristic_;


  Eigen::Matrix4f local_grid_to_base_link_;
  Eigen::Matrix4f map_to_base_link_;
};

}  // namespace aut_local_planner

#endif  // AUT_LOCAL_PLANNER_LOCAL_GRID_H_