#include "open_space_map/open_space_map.h"
OpenSpaceMap::OpenSpaceMap(/* args */) {}

OpenSpaceMap::~OpenSpaceMap() {}

void OpenSpaceMap::PlotAll() {
  PlotObstacles(obstacles_vertices_vec_);
  PlotObstacles(swelling_obstacles_vec_);

  PlotTrajectory(coarse_trajectory_);

  PlotDrivingBound(f_bound_);
  // PlotDrivingBound(b_bound_);

  PlotTrajectory(optimized_trajectory_);

  plt::show(); // plot::show()是一个阻塞函数
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
    std::map<std::string, std::string> keywords;
    keywords["color"] = "grey";
    plt::fill(x, y, keywords);
  }
}

void OpenSpaceMap::PlotTrajectory(const DiscretizedTrajectory &trajectory) {

  std::vector<double> x, y;
  for (const auto point : trajectory) {
    x.push_back(point.x());
    y.push_back(point.y());
  }

  plt::plot(x, y);
}

void OpenSpaceMap::PlotDrivingBound(const Eigen::MatrixXd bound_) {
  //(x_min,x_max,y_min,y_max)

  int size = bound_.cols();
  for (int i = 0; i < size; i++) {
    std::vector<double> x, y;
    //左下
    x.push_back(bound_(0, i));
    y.push_back(bound_(2, i));
    //左上
    x.push_back(bound_(0, i));
    y.push_back(bound_(3, i));
    //右上
    x.push_back(bound_(1, i));
    y.push_back(bound_(3, i));
    //右下
    x.push_back(bound_(1, i));
    y.push_back(bound_(2, i));

    //左下
    x.push_back(bound_(0, i));
    y.push_back(bound_(2, i));

    if ((i % 10) == 0) {
      plt::plot(x, y);
    }
  }
}

void OpenSpaceMap::SetFrontDrivingBound(Eigen::MatrixXd f_bound) {
  f_bound_ = f_bound;
}

void OpenSpaceMap::SetBackDrivingBound(Eigen::MatrixXd b_bound) {
  b_bound_ = b_bound;
}

void OpenSpaceMap::SetSwellingObstacle(
    std::vector<std::vector<common::math::Vec2d>> swelling_obstacles_vec) {
  swelling_obstacles_vec_ = swelling_obstacles_vec;
}