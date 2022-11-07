#include "open_space_map/open_space_map.h"
OpenSpaceMap::OpenSpaceMap(/* args */) {}

OpenSpaceMap::~OpenSpaceMap() {}

void OpenSpaceMap::PlotAll() {
  PlotObstacles(obstacles_vertices_vec_);

  // PlotSwellingObstacles(swelling_obstacles_vec_);

  PlotTrajectory(coarse_trajectory_, "k");

  PlotDrivingBound(f_bound_);
  // PlotDrivingBound(b_bound_);

  PlotTrajectory(optimized_trajectory_, "b");

  plt::figure(2);
  PlotTrajectoryV(coarse_trajectory_, "k");
  PlotTrajectoryV(optimized_trajectory_, "b");

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

void OpenSpaceMap::PlotSwellingObstacles(
    const std::vector<std::vector<common::math::Vec2d>>
        &obstacles_vertices_vec) {
  for (const auto &obs_vertices : obstacles_vertices_vec) {
    std::vector<double> x, y;
    for (const auto &obs_vertice : obs_vertices) {
      x.push_back(obs_vertice.x());
      y.push_back(obs_vertice.y());
    }
    std::map<std::string, std::string> keywords;

    //   keywords["alpha"] = "0.4";
    keywords["color"] = "grey";
    plt::fill(x, y, keywords);
  }
}

void OpenSpaceMap::PlotTrajectory(const DiscretizedTrajectory &trajectory,
                                  const std::string &color) {

  std::vector<double> x, y;
  for (const auto point : trajectory) {
    x.push_back(point.path_point().x());
    y.push_back(point.path_point().y());
  }

  plt::plot(x, y, color);
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

    if ((i % 20) == 0) {
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

void OpenSpaceMap::SwellingObstacle(
    const std::vector<common::math::Vec2d> &obstacles_vertices, const double &r,
    std::vector<common::math::Vec2d> *swelling_obstacles_vertices) {
  std::vector<common::math::Vec2d> temp_obstacles_vertices = obstacles_vertices;

  //将第一个点放在末尾，将末尾点放在头部使每个点都有相邻点
  common::math::Vec2d first_vertice = obstacles_vertices.front();
  common::math::Vec2d last_vertice = obstacles_vertices.back();
  temp_obstacles_vertices.insert(temp_obstacles_vertices.begin(), last_vertice);
  temp_obstacles_vertices.push_back(first_vertice);

  int size = temp_obstacles_vertices.size();
  // zhaokun20221013
  for (int i = 1; i < size - 1; i++) {
    common::math::Vec2d left_cur =
        (temp_obstacles_vertices[i] - temp_obstacles_vertices[i - 1]);
    left_cur = left_cur / left_cur.Length(); // left->cur单位矢量
    common::math::Vec2d right_cur =
        (temp_obstacles_vertices[i] - temp_obstacles_vertices[i + 1]);
    right_cur = right_cur / right_cur.Length(); // right->cur单位矢量
    double sin_theta = left_cur.CrossProd(right_cur);
    double d_cur_swell = 0; //计算sinθ
    if (sin_theta == 0) {
      std::cout << "障碍物顶点错误..." << std::endl;
      continue;
    } else {
      d_cur_swell = r / sin_theta;
    }
    common::math::Vec2d cur_swel =
        d_cur_swell * (left_cur + right_cur); //膨胀方向,变成单位相邻*d
                                              // zhaokun20221012发生错误

    common::math::Vec2d swelling_obstacles_vertice =
        temp_obstacles_vertices[i] + cur_swel;
    //一开始vector为空时，不能对其进行下标赋值。而要用push_back().假如你先设置其大小是可以的.
    // v.resiz(10); v[0] =1
    (*swelling_obstacles_vertices).push_back(swelling_obstacles_vertice);
  }
}

void OpenSpaceMap::SwellingObstacles(const double &r) {
  for (const auto &obs : obstacles_vertices_vec_) {
    std::vector<common::math::Vec2d> swelling_obstacles_vertices;
    SwellingObstacle(obs, r, &swelling_obstacles_vertices); //膨胀障碍物
    swelling_obstacles_vec_.push_back(swelling_obstacles_vertices);
  }
}

const std::vector<std::vector<common::math::Vec2d>>
OpenSpaceMap::swelling_obstacles_vec() const {
  return swelling_obstacles_vec_;
}

void OpenSpaceMap::PlotTrajectoryV(const DiscretizedTrajectory &trajectory,
                                   const std::string &color) {

  std::vector<double> x, y;
  for (const auto point : trajectory) {
    x.push_back(point.relative_time());
    y.push_back(point.v());
  }
  plt::plot(x, y, color);
}