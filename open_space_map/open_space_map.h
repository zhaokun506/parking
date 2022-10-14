/*map的作用
    //设置障碍物
    绘制图内的图形
*/
#pragma once

#include "common/math/vec2d.h"
#include "common/matplot/matplotlibcpp.h"
#include "eigen3/Eigen/Dense"
#include "planning_data/trajectory/discretized_trajectory.h"
#include <map>
#include <string>
#include <vector>

namespace plt = matplotlibcpp;
// using Eigen::MatrixXd;
class MapPoint {
public:
  MapPoint(double x, double y, double phi) : x(x), y(y), phi(phi){};
  ~MapPoint() = default;
  double x;
  double y;
  double phi;
};

class OpenSpaceMap {

public:
  OpenSpaceMap(/* args */);
  ~OpenSpaceMap();

  void PlotAll();

  void SetXYBounds(double x_min, double x_max, double y_min, double y_max);
  void SetOnebstacle(std::vector<common::math::Vec2d> obstacles_vertices);
  void SetSwellingObstacle(
      std::vector<std::vector<common::math::Vec2d>> swelling_obstacles_vec);
  void SetOptimizedTrajectory(
      DiscretizedTrajectory optimized_trajectory); //轨迹点即可
  void SetCoarseTrajectory(DiscretizedTrajectory coarse_trajectory);
  void SetFrontDrivingBound(Eigen::MatrixXd f_bound);
  void SetBackDrivingBound(Eigen::MatrixXd b_bound);

  const std::vector<std::vector<common::math::Vec2d>>
  obstacles_vertices_vec() const;
  const std::vector<double> XYbounds() const;

private:
  void PlotObstacles(const std::vector<std::vector<common::math::Vec2d>>
                         &obstacles_vertices_vec_);
  void PlotTrajectory(const DiscretizedTrajectory &trajectory);

  void PlotDrivingBound(const Eigen::MatrixXd bound_);

private:
  /* data */
  // obstacles_vertices_vec_ in clock wise顺时针 order. Take different
  // approach towards warm start and distance approach
  std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec_;
  std::vector<std::vector<common::math::Vec2d>> swelling_obstacles_vec_;

  // x_min,x_max,y_min,y_max
  std::vector<double> XYbounds_;

  DiscretizedTrajectory optimized_trajectory_;
  DiscretizedTrajectory coarse_trajectory_;

  Eigen::MatrixXd f_bound_;
  Eigen::MatrixXd b_bound_;
};

/*
const放在函数后主要是限制类中的成员函数，const放在函数前是限制函数返回类型为指针时通过指针对返回值的修改。

const放在函数后通常是用在类中，限制类的成员函数不能对成员变量进行修改。同时，被函数后的const修饰的成员函数也不能调用其他非const的成员函数。
*/