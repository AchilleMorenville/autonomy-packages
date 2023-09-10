#ifndef AUT_LOCAL_PLANNER_HYBRID_A_STAR_H_
#define AUT_LOCAL_PLANNER_HYBRID_A_STAR_H_

#include <vector>
#include <queue>

#include <Eigen/Core>

namespace aut_local_planner {

struct MotionPrimitive {
  float dx;
  float dy;
  float dtheta;

  MotionPrimitive();
  MotionPrimitive(float dx, float dy, float dtheta);
  MotionPrimitive(const MotionPrimitive& mp);
  MotionPrimitive& operator=(MotionPrimitive mp);
  ~MotionPrimitive();
};

class HybridAStar {
 public:
  HybridAStar();
  void SetGrid(int x_dim, int y_dim, float cell_size, int theta_dim, 
               const std::vector<float>& grid, 
               const Eigen::Matrix4f& grid_to_base_link, 
               const Eigen::Matrix4f& map_to_base_link);
  bool SetTarget(const Eigen::Matrix4f& map_to_target);
  bool GetMotion(MotionPrimitive& motion);
 
 private:
  struct Pose {
    float x;
    float y;
    float theta;

    Pose();
    explicit Pose(const Eigen::Matrix4f& grid_to_pose);
    Pose(float x, float y, float theta);
    Pose(const Pose& p);
    Pose& operator=(Pose p);
    Pose operator+(const MotionPrimitive& motion_primitive) const;
    ~Pose();
  };

  struct State {
    Pose pose;
    int id;
    int parent_id;
    MotionPrimitive motion_from_parent;
    float g;
    float h;

    State();
    State(Pose pose, int id, int parent_id, MotionPrimitive motion_from_parent, 
          float g, float h);
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

  bool InsideGrid(const Pose& pose);
  bool IsFree(const Pose& pose);
  int GetIndex(const Pose& pose);
  int GetIndex2D(const Pose& pose);
  void PrecomputeDijkstraHeuristic();
  float GetDijkstraHeuristic(const Pose& pose);
  bool IsTarget(const State& state);
  void GetNeighbors(const State& state, std::vector<State>& neighbors);
  bool Accessible(const Pose& pose);
  float GetCost(const Pose& pose, const MotionPrimitive& previous_motion, const MotionPrimitive& current_motion);

  // Parameters
  bool precise_target_;

  int x_dim_;
  int y_dim_;
  float cell_size_;
  int theta_dim_;
  float min_dist_;
  std::vector<MotionPrimitive> motion_primitives_;

  float cost_forward_;
  float cost_backward_;
  float cost_side_;
  float cost_standing_;
  float cost_turning_;
  float cost_change_of_direction_;

  // State
  std::vector<float> grid_;
  std::vector<float> dijkstra_grid_;
  Eigen::Matrix4f grid_to_base_link_;
  Eigen::Matrix4f map_to_base_link_;
  Eigen::Matrix4f map_to_grid_;
  Eigen::Matrix4f map_to_target_;
  Pose target_pose_;
}; 

}

#endif  // AUT_LOCAL_PLANNER_HYBRID_A_STAR_H_