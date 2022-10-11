#include "open_space_map/open_space_map.h"

OpenSpaceMap::OpenSpaceMap(/* args */) {}

OpenSpaceMap::~OpenSpaceMap() {}

void OpenSpaceMap::PlotAll() {
  PlotObstacles(obstacles_vertices_vec_);
  PlotTrajectory(coarse_trajectory_);
  PlotTrajectory(optimized_trajectory_);
}

void OpenSpaceMap::SetXYBounds(double x_min, double x_max, double y_min,
                               double y_max) {
  XYbounds_.push_back(x_min);
  XYbounds_.push_back(x_max);
  XYbounds_.push_back(y_min);
  XYbounds_.push_back(y_max);
}
void OpenSpaceMap::SetOnebstacle(
    std::vector<common::math::Vec2d> obstacles_vertices) {
  obstacles_vertices_vec_.push_back(obstacles_vertices);
}
void OpenSpaceMap::SetOptimizedTrajectory(
    DiscretizedTrajectory optimized_trajectory) {
  optimized_trajectory_ = optimized_trajectory;
}
//轨迹点即可
void OpenSpaceMap::SetCoarseTrajectory(
    DiscretizedTrajectory coarse_trajectory) {
  coarse_trajectory_ = coarse_trajectory;
}

const std::vector<std::vector<common::math::Vec2d>>
OpenSpaceMap::obstacles_vertices_vec() const {
  return obstacles_vertices_vec_;
}
const std::vector<double> OpenSpaceMap::XYbounds() const { return XYbounds_; }

void OpenSpaceMap::PlotObstacles(
    const std::vector<std::vector<common::math::Vec2d>>
        &obstacles_vertices_vec) {
  for (const auto &obs_vertices : obstacles_vertices_vec) {
    std::vector<double> x, y;
    for (const auto &obs_vertice : obs_vertices) {
      x.push_back(obs_vertice.x());
      y.push_back(obs_vertice.y());
    }

    // plt::fill(x, y);
  }
}

void OpenSpaceMap::PlotTrajectory(const DiscretizedTrajectory &trajectory) {}