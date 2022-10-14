#include "common/math/vec2d.h"
#include "common/pnc_point.h"
#include "configs/open_space_trajectory_optimizer_config.h"
#include "configs/vehicle_config.h"
#include "open_space_map/open_space_map.h"
#include "planning_data/trajectory/discretized_trajectory.h"
#include "trajectory_smoother/construct_driving_corridor.h"
#include <memory>
#include <vector>

using namespace common::math;
int main() {

  std::cout << "hello" << std::endl;

  //设置地图边界
  std::unique_ptr<OpenSpaceMap> map = std::make_unique<OpenSpaceMap>();
  map->SetXYBounds(0, 100, 0, 100);
  //设置障碍物信息，逆时针顶点
  std::vector<Vec2d> obs1, obs2, obs3, obs4, obs5, obs6;

  obs1.push_back(Vec2d(30, 48));
  obs1.push_back(Vec2d(30, 36));
  obs1.push_back(Vec2d(2, 36));
  obs1.push_back(Vec2d(2, 48));
  map->SetOnebstacle(obs1);

  obs2.push_back(Vec2d(95, 42));
  obs2.push_back(Vec2d(95, 6));
  obs2.push_back(Vec2d(32, 6));
  obs2.push_back(Vec2d(52, 42));
  map->SetOnebstacle(obs2);

  obs3.push_back(Vec2d(55, 53));
  obs3.push_back(Vec2d(55, 43));
  obs3.push_back(Vec2d(45, 43));
  obs3.push_back(Vec2d(45, 53));
  map->SetOnebstacle(obs3);

  obs4.push_back(Vec2d(25, 68));
  obs4.push_back(Vec2d(30, 55));
  obs4.push_back(Vec2d(16, 70));
  map->SetOnebstacle(obs4);

  obs5.push_back(Vec2d(44, 100));
  obs5.push_back(Vec2d(44, 96));
  obs5.push_back(Vec2d(42, 96));
  obs5.push_back(Vec2d(42, 100));
  map->SetOnebstacle(obs5);

  obs6.push_back(Vec2d(38, 100));
  obs6.push_back(Vec2d(38, 96));
  obs6.push_back(Vec2d(36, 96));
  obs6.push_back(Vec2d(36, 100));
  map->SetOnebstacle(obs6);

  // map->PlotAll();

  //设置起点终点位置

  auto XYbounds = map->XYbounds();
  auto obstacles_vertices_vec = map->obstacles_vertices_vec();

  PlannerOpenSpaceConfig open_space_config;
  VehicleParam vehicle_config;
  std::unique_ptr<ConstructDrivingCorridor> cor =
      std::make_unique<ConstructDrivingCorridor>(open_space_config,
                                                 vehicle_config);

  Eigen::MatrixXd xWS = Eigen::MatrixXd::Zero(4, 1);
  xWS << 33, 42, 1.6, 0;

  Eigen::MatrixXd f_bound = Eigen::MatrixXd::Zero(4, 1);
  Eigen::MatrixXd b_bound = Eigen::MatrixXd::Zero(4, 1);
  cor->Construct(
      obstacles_vertices_vec, //障碍物顶点数组，顶点的表示法为Vec2d向量,
      xWS,                    //初始路径的
      1,                      //离散点数
      &f_bound, // 1.车辆前圆心可行驶边界4*n矩阵,(x_min,x_max,y_min,y_max)'*n
      &b_bound);

  map->SetSwellingObstacle(cor->swelling_obstacles_vec());

  map->SetFrontDrivingBound(f_bound);
  map->SetBackDrivingBound(b_bound);
  map->PlotAll();

  return 1;
}