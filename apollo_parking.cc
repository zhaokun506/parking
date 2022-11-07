#include "common/math/vec2d.h"
#include "configs/open_space_trajectory_optimizer_config.h"
#include "eigen3/Eigen/Dense"
#include "open_space_map/open_space_map.h"
#include "open_space_trajectory_optimizer/apollo_trajectory_optimizer.h"
#include "trajectory_smoother/open_space_roi_deal.h"
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
  // //随机障碍物
  // obs1.push_back(Vec2d(30, 48));
  // obs1.push_back(Vec2d(30, 46));
  // obs1.push_back(Vec2d(20, 36));
  // obs1.push_back(Vec2d(20, 48));
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

  // //侧方停车场景
  // obs1.push_back(Vec2d(2, 10));
  // obs1.push_back(Vec2d(2, 3));
  // obs1.push_back(Vec2d(30, 3));
  // obs1.push_back(Vec2d(30, 10));
  // map->SetOnebstacle(obs1);
  // //车2
  // obs2.push_back(Vec2d(37, 10));
  // obs2.push_back(Vec2d(37, 3));
  // obs2.push_back(Vec2d(60, 3));
  // obs2.push_back(Vec2d(60, 10));
  // map->SetOnebstacle(obs2);
  // //房子
  // obs3.push_back(Vec2d(0, 20));
  // obs3.push_back(Vec2d(0, 16));
  // obs3.push_back(Vec2d(60, 16));
  // obs3.push_back(Vec2d(60, 20));
  // map->SetOnebstacle(obs3);

  // obs4.push_back(Vec2d(30, 7.5));
  // obs4.push_back(Vec2d(30, 0));
  // obs4.push_back(Vec2d(37, 0));
  // obs4.push_back(Vec2d(37, 7.5));
  // map->SetOnebstacle(obs4);

  //倒车入库场景 逆时针
  obs1.push_back(Vec2d(2, 10));
  obs1.push_back(Vec2d(2, 3));
  obs1.push_back(Vec2d(30, 3));
  obs1.push_back(Vec2d(30, 10));
  map->SetOnebstacle(obs1);
  //车2
  obs2.push_back(Vec2d(32.5, 10));
  obs2.push_back(Vec2d(32.5, 3));
  obs2.push_back(Vec2d(60, 3));
  obs2.push_back(Vec2d(60, 10));
  map->SetOnebstacle(obs2);
  //房子
  obs3.push_back(Vec2d(0, 20));
  obs3.push_back(Vec2d(0, 16));
  obs3.push_back(Vec2d(60, 16));
  obs3.push_back(Vec2d(60, 20));
  map->SetOnebstacle(obs3);

  // //倒车入库场景 顺时针
  // obs1.push_back(Vec2d(2, 10));
  // obs1.push_back(Vec2d(30, 10));
  // obs1.push_back(Vec2d(30, 3));
  // obs1.push_back(Vec2d(2, 3));

  // map->SetOnebstacle(obs1);
  // //车2
  // obs2.push_back(Vec2d(34, 10));
  // obs2.push_back(Vec2d(60, 10));
  // obs2.push_back(Vec2d(60, 3));
  // obs2.push_back(Vec2d(34, 3));

  // map->SetOnebstacle(obs2);
  // // //房子
  // obs3.push_back(Vec2d(0, 20));
  // obs3.push_back(Vec2d(60, 20));
  // obs3.push_back(Vec2d(60, 16));
  // obs3.push_back(Vec2d(0, 16));

  // map->SetOnebstacle(obs3);

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
  //   MapPoint end_pose(28.5, 57 + 1.043 - 4.933 / 2,
  //                     -1 / 2.0 * M_PI); //双精度运算避免写正数，导致取整

  auto XYbounds = map->XYbounds();

  auto swelling_obstacles_vec = map->swelling_obstacles_vec();
  auto obstacles_vertices_vec = map->obstacles_vertices_vec();
  //障碍物膨胀应该在外边做

  double rotate_angle = 0;
  common::math::Vec2d translate_origin(0, 0);
  // Eigen::MatrixXi obstacles_edges_num(1, 1);
  OpenSpaceTrajectoryOptimizerConfig open_space_config;

  std::vector<common::TrajectoryPoint> stitching_trajectory;
  common::TrajectoryPoint point1;
  point1.mutable_path_point()->set_x(5);
  point1.mutable_path_point()->set_y(12);
  point1.mutable_path_point()->set_theta(0);
  point1.set_v(0);
  point1.set_a(0);
  point1.set_da(0);
  point1.set_relative_time(0);
  point1.set_steer(0);
  stitching_trajectory.push_back(point1);

  std::vector<double> end_pose;
  // 倒车终点
  end_pose.push_back(31.25);
  end_pose.push_back(7.5 - (4.933 / 2 - 1.043));
  end_pose.push_back(1 / 2.0 * M_PI);
  end_pose.push_back(0);

  // //侧方终点
  // end_pose.push_back(33 - ((4.933 / 2 - 1.043)));
  // end_pose.push_back(8.75);
  // end_pose.push_back(0);
  // end_pose.push_back(0);

  size_t obstacles_num = obstacles_vertices_vec.size();
  int obs_edges_num = 0;
  Eigen::MatrixXi obstacles_edges_num = Eigen::MatrixXi::Zero(obstacles_num, 1);
  for (int i = 0; i < obstacles_num; i++) {
    obstacles_edges_num(i, 0) = obstacles_vertices_vec[i].size();
  }

  std::unique_ptr<OpenSpaceRoiDecider> roi_decider =
      std::make_unique<OpenSpaceRoiDecider>();

  Eigen::MatrixXd obstacles_A =
      Eigen::MatrixXd::Zero(obstacles_edges_num.sum(), 2);
  Eigen::MatrixXd obstacles_b =
      Eigen::MatrixXd::Zero(obstacles_edges_num.sum(), 1);
  // Ax>b,什么不是
  roi_decider->GetHyperPlanes(obstacles_num, obstacles_edges_num,
                              obstacles_vertices_vec, &obstacles_A,
                              &obstacles_b);

  for (int i = 0; i < obstacles_A.rows(); i++) {
    std::cout << obstacles_A(i, 0) << "," << obstacles_A(i, 1) << "    "
              << obstacles_b(i, 0) << std::endl;
  }

  double time_latency;
  std::unique_ptr<ApolloOpenSpaceTrajectoryOptimizer> optimizer =
      std::make_unique<ApolloOpenSpaceTrajectoryOptimizer>(open_space_config);

  // double time_latency;
  auto plan_start_time = clock();
  optimizer->Plan(stitching_trajectory, end_pose,
                  XYbounds, // 4,XYbounds in xmin, xmax, ymin, ymax
                  rotate_angle, translate_origin,
                  obstacles_edges_num, // H表示中A和b矩阵维数所需的维数
                  obstacles_A * (-1), obstacles_b * (-1),
                  obstacles_vertices_vec, //以逆时钟顺序存储障碍物顶点的向量
                  &time_latency);
  std::cout << "plan_start_time耗时:"
            << (clock() - plan_start_time) * 1.0 / CLOCKS_PER_SEC << "s"
            << std::endl;

  DiscretizedTrajectory coarse_trajectory;
  DiscretizedTrajectory optimized_trajectory;
  optimizer->GetCoarseTrajectory(&coarse_trajectory);
  map->SetCoarseTrajectory(coarse_trajectory);

  optimizer->GetOptimizedTrajectory(&optimized_trajectory);
  map->SetOptimizedTrajectory(optimized_trajectory);

  map->PlotAll();

  return 1;
}