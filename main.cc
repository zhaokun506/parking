#include "common/math/vec2d.h"
#include "configs/open_space_trajectory_optimizer_config.h"
#include "eigen3/Eigen/Dense"
#include "open_space_map/open_space_map.h"
#include "open_space_trajectory_optimizer/open_space_trajectory_optimizer.h"
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
  //随机障碍物
  // obs1.push_back(Vec2d(30, 48));
  // obs1.push_back(Vec2d(30, 36));
  // obs1.push_back(Vec2d(2, 36));
  // obs1.push_back(Vec2d(2, 48));
  // map->SetOnebstacle(obs1);

  // obs2.push_back(Vec2d(95, 42));
  // obs2.push_back(Vec2d(95, 6));
  // obs2.push_back(Vec2d(32, 6));
  // obs2.push_back(Vec2d(52, 42));
  // map->SetOnebstacle(obs2);

  // obs3.push_back(Vec2d(55, 53));
  // obs3.push_back(Vec2d(55, 43));
  // obs3.push_back(Vec2d(45, 43));
  // obs3.push_back(Vec2d(45, 53));
  // map->SetOnebstacle(obs3);

  // obs4.push_back(Vec2d(25, 68));
  // obs4.push_back(Vec2d(30, 55));
  // obs4.push_back(Vec2d(16, 70));
  // map->SetOnebstacle(obs4);

  // obs5.push_back(Vec2d(46, 100));
  // obs5.push_back(Vec2d(46, 96));
  // obs5.push_back(Vec2d(44, 96));
  // obs5.push_back(Vec2d(44, 100));
  // map->SetOnebstacle(obs5);

  // obs6.push_back(Vec2d(36, 100));
  // obs6.push_back(Vec2d(36, 96));
  // obs6.push_back(Vec2d(34, 96));
  // obs6.push_back(Vec2d(34, 100));
  // map->SetOnebstacle(obs6);

  /*
    //侧方停车场景
    //车1
    obs1.push_back(Vec2d(27 + 2, 58.75 + 1));
    obs1.push_back(Vec2d(27 + 2, 58.75 - 1));
    obs1.push_back(Vec2d(27 - 32, 58.75 - 1));
    obs1.push_back(Vec2d(27 - 32, 58.75 + 1));
    map->SetOnebstacle(obs1);
    //车2
    obs2.push_back(Vec2d(40+ 2, 58.75 + 1));
    obs2.push_back(Vec2d(40 + 2, 58.75 - 1));
    obs2.push_back(Vec2d(40 - 2, 58.75 - 1));
    obs2.push_back(Vec2d(40 - 2, 58.75 + 1));
    map->SetOnebstacle(obs2);
    //房子
    obs3.push_back(Vec2d(60, 40));
    obs3.push_back(Vec2d(60, 0));
    obs3.push_back(Vec2d(5, 0));
    obs3.push_back(Vec2d(5, 40));
    map->SetOnebstacle(obs3);
  */

  //倒车入库场景
  // obs1.push_back(Vec2d(26 + 1.25, 57 + 3));
  // obs1.push_back(Vec2d(26 + 1.25, 57 - 3));
  // obs1.push_back(Vec2d(26 - 1.25, 57 - 3));
  // obs1.push_back(Vec2d(26 - 1.25, 57 + 3));
  // map->SetOnebstacle(obs1);
  // //车2
  // obs2.push_back(Vec2d(31 + 1.25, 57 + 3));
  // obs2.push_back(Vec2d(31 + 1.25, 57 - 3));
  // obs2.push_back(Vec2d(31 - 1.25, 57 - 3));
  // obs2.push_back(Vec2d(31 - 1.25, 57 + 3));
  // map->SetOnebstacle(obs2);
  //  //房子
  //  obs3.push_back(Vec2d(60, 40));
  //  obs3.push_back(Vec2d(60, 0));
  //  obs3.push_back(Vec2d(5, 0));
  //  obs3.push_back(Vec2d(5, 40));
  //  map->SetOnebstacle(obs3);

  //膨胀障碍物
  double swelling_r =
      pow((2.11 / 2.0) * (2.11 / 2.0) + (4.99 / 4.0) * (4.99 / 4.0), 0.5);
  map->SwellingObstacles(swelling_r * 0.6);

  // map->PlotAll();

  //设置起点终点位置
  MapPoint start_pose(2, 2, 0);

  //侧方停车
  // MapPoint end_pose(40, 90,
  //                   1 / 2.0 * M_PI); //双精度运算避免写正数，导致取整
  // MapPoint end_pose(32 + 1.043 - 4.933 / 2, 58.75,
  //                   0); //双精度运算避免写正数，导致取整
  MapPoint end_pose(28.5, 57 + 1.043 - 4.933 / 2,
                    -1 / 2.0 * M_PI); //双精度运算避免写正数，导致取整

  auto XYbounds = map->XYbounds();

  auto swelling_obstacles_vec = map->swelling_obstacles_vec();
  auto obstacles_vertices_vec = map->obstacles_vertices_vec();
  //障碍物膨胀应该在外边做

  double rotate_angle = 0;
  common::math::Vec2d translate_origin(0, 0);
  Eigen::MatrixXi obstacles_edges_num(1, 1);
  OpenSpaceTrajectoryOptimizerConfig open_space_config;
  double time_latency;
  std::unique_ptr<OpenSpaceTrajectoryOptimizer> optimizer =
      std::make_unique<OpenSpaceTrajectoryOptimizer>(open_space_config);

  optimizer->Plan(start_pose, //起点
                  end_pose,
                  XYbounds, // 4,XYbounds in xmin, xmax, ymin, ymax
                  rotate_angle, translate_origin,
                  obstacles_edges_num, // H表示中A和b矩阵维数所需的维数
                  swelling_obstacles_vec, //以逆时钟顺序存储障碍物顶点的向量
                  &time_latency);

  DiscretizedTrajectory coarse_trajectory;
  DiscretizedTrajectory optimized_trajectory;

  optimizer->GetCoarseTrajectory(&coarse_trajectory);
  optimizer->GetOptimizedTrajectory(&optimized_trajectory);

  map->SetSwellingObstacle(
      optimizer->GetConstructCorridorPtr()->swelling_obstacles_vec());

  map->SetFrontDrivingBound(optimizer->GetFrontDrivingBound());
  map->SetBackDrivingBound(optimizer->GetBackDrivingBound());
  map->SetCoarseTrajectory(coarse_trajectory);
  map->SetOptimizedTrajectory(optimized_trajectory);

  map->PlotAll();

  return 1;
}