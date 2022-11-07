/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "open_space_trajectory_optimizer/open_space_trajectory_optimizer.h"

#include <utility>

using common::math::Vec2d;

OpenSpaceTrajectoryOptimizer::OpenSpaceTrajectoryOptimizer(
    const OpenSpaceTrajectoryOptimizerConfig &config)
    : config_(config) {
  // Load config加载配置
  // config_ = config;
  // Initialize hybrid astar class pointer初始化混合A*指针，生成粗略初始轨迹
  warm_start_.reset(new HybridAStar(config_.planner_open_space_config()));
  // 构建行车隧道
  construct_corridor_.reset(new ConstructDrivingCorridor(
      config_.planner_open_space_config(), vehicle_param_));
  // Initialize distance approach trajectory smootherclass
  // pointer初始化距离接近优化算法指针
  distance_approach_.reset(
      new DistanceApproachProblem(config_.planner_open_space_config()));
}

int OpenSpaceTrajectoryOptimizer::Plan(
    const MapPoint &start_pose, //起点
    const MapPoint &end_pose,   //终点
    const std::vector<double> &XYbounds, double rotate_angle,
    const common::math::Vec2d &translate_origin,
    const Eigen::MatrixXi &obstacles_edges_num,
    const std::vector<std::vector<common::math::Vec2d>>
        &obstacles_vertices_vec, //以逆时钟顺序存储障碍物顶点的向量
    double *time_latency) {
  //判断输入数据是否正确
  if (XYbounds.empty()) {
    std::cout << "OpenSpaceTrajectoryOptimizer input data not ready";
    return 0;
  }

  // const auto start_timestamp = std::chrono::system_clock::now();

  // init x, y, z would be rotated.
  double init_x = start_pose.x;
  double init_y = start_pose.y;
  double init_phi = start_pose.phi;
  double end_x = end_pose.x;
  double end_y = end_pose.y;
  double end_phi = end_pose.phi;

  // Result container for warm start (initial velocity is assumed to be 0 for
  // now)初始速度读假设为0
  HybridAStartResult result;
  //☆☆☆☆☆☆☆☆☆☆☆1.调用混合A*算法☆☆☆☆☆☆☆☆☆☆☆
  if (warm_start_->Plan(init_x, init_y, init_phi, end_x, end_y, end_phi,
                        XYbounds, obstacles_vertices_vec, &result)) {
    std::cout << "State warm start problem solved successfully!" << std::endl;
  } else {
    std::cout << "State warm start problem failed to solve" << std::endl;
    return 0;
  }

  // Containers for distance approach trajectory smoothing problem
  //定义输入输出矩阵
  Eigen::MatrixXd xWS;
  Eigen::MatrixXd uWS;

  Eigen::MatrixXd state_result_ds;   // 4(n+1)
  Eigen::MatrixXd control_result_ds; // 2n
  Eigen::MatrixXd time_result_ds;    // 1

  //将混合A*的结果载入到eigen
  LoadHybridAstarResultInEigen(&result, &xWS, &uWS);

  const double init_steer = 0;
  const double init_a = 0;
  Eigen::MatrixXd last_time_u(2, 1);
  last_time_u << init_steer, init_a;

  const double init_v = 0;
  //☆☆☆☆☆☆☆☆☆☆☆2.热启动程序和优化程序☆☆☆☆☆☆☆☆☆☆☆
  /*xWs                    状态量矩阵
    uWs                    控制量矩阵
    XYbounds               xy边界
    obstacles_edges_num    障碍物边的
    obstacles_A
    obstacles_b
    obstacles_vertices_vec
    last_time_u            终点控制量状态
    init_v

    state_result_ds        状态量优化结果
    control_result_ds       控制量优化结果
    time_result_ds         时间优化结果dt

  */

  //生成行车隧道
  int num_point = xWS.cols();
  // l 矩阵需要初始化
  /*
  x0 x1...xn
  y0 y1...yn
  */
  Eigen::MatrixXd f_driving_bound = Eigen::MatrixXd::Zero(4, num_point); // 4*n
  Eigen::MatrixXd b_driving_bound = Eigen::MatrixXd::Zero(4, num_point); // 4*n
  construct_corridor_->Construct(
      obstacles_vertices_vec, //障碍物顶点数组，顶点的表示法为Vec2d向量,
      xWS,                    //初始路径
      num_point,              //离散点数
      &f_driving_bound, // 1.车辆前圆心可行驶边界n*4矩阵,n*(x_min,x_max,y_min,y_max)
      &b_driving_bound);

  f_bound_ = f_driving_bound;
  r_bound_ = b_driving_bound;

  // void GetCoarseTrajectory(DiscretizedTrajectory * optimized_trajectory);
  // zhaokun
  time_result_ds = Eigen::MatrixXd::Zero(1, num_point);
  for (int i = 0; i < num_point; i++) {
    time_result_ds(0, i) = i * 1; // dt ts应该是固定的
  }

  GenerateCoarseTraj(
      xWS, uWS, XYbounds, obstacles_edges_num, f_driving_bound,
      b_driving_bound, //此处将障碍物的两个矩阵替换为可行驶隧道的两个矩阵
      obstacles_vertices_vec, last_time_u, init_v, &state_result_ds,
      &control_result_ds, &time_result_ds);

  LoadCoarseTrajectory(state_result_ds, control_result_ds, time_result_ds);

  if (!GenerateDistanceApproachTraj(
          xWS, uWS, XYbounds, obstacles_edges_num, f_driving_bound,
          b_driving_bound, //此处将障碍物的两个矩阵替换为可行驶隧道的两个矩阵
          obstacles_vertices_vec, last_time_u, init_v, &state_result_ds,
          &control_result_ds, &time_result_ds)) {
    std::cout << "distance approach smoothing problem failed to solve"
              << std::endl;
    return 0;
  }

  LoadTrajectory(state_result_ds, control_result_ds, time_result_ds);

  // const auto end_timestamp = std::chrono::system_clock::now();
  //  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  // std::cout << "open space trajectory smoother total time: "
  //           << diff.count() * 1000.0 << " ms.";
  //*time_latency = diff.count() * 1000.0;

  return 1;
}

void OpenSpaceTrajectoryOptimizer::LoadTrajectory(
    const Eigen::MatrixXd &state_result, const Eigen::MatrixXd &control_result,
    const Eigen::MatrixXd &time_result) {
  optimized_trajectory_.clear();

  // Optimizer doesn't take end condition control state into consideration for
  // now
  size_t states_size = state_result.cols();
  size_t times_size = time_result.cols();
  size_t controls_size = control_result.cols();

  double relative_time = 0.0;
  double relative_s = 0.0;
  Vec2d last_path_point(state_result(0, 0), state_result(1, 0));
  std::cout << time_result(0, 0) << std::endl;
  for (size_t i = 0; i < states_size; ++i) {
    common::TrajectoryPoint point;//此处apollo使用的是pathpoint
    point.set_x(state_result(0, i));
    point.set_y(state_result(1, i));
    point.set_theta(state_result(2, i));
    point.set_v(state_result(3, i));
    Vec2d cur_path_point(state_result(0, i), state_result(1, i));
    relative_s += cur_path_point.DistanceTo(last_path_point);
    point.set_s(relative_s);
    // TODO(Jinyun): Evaluate how to set end states control input
    if (i == controls_size) {
      point.set_steer(0.0);
      point.set_a(0.0);
    } else {
      point.set_steer(control_result(0, i));
      point.set_a(control_result(1, i));
    }

    if (i == 0) {
      point.set_relative_time(relative_time);
    } else {
      relative_time = time_result(0, 0) * i;
      point.set_relative_time(relative_time);
    }

    optimized_trajectory_.emplace_back(point);
    last_path_point = cur_path_point;
  }
}

void OpenSpaceTrajectoryOptimizer::UseWarmStartAsResult(
    const Eigen::MatrixXd &xWS, const Eigen::MatrixXd &uWS,
    Eigen::MatrixXd *state_result_ds, Eigen::MatrixXd *control_result_ds,
    Eigen::MatrixXd *time_result_ds) {
  std::cout << "Use warm start as trajectory output" << std::endl;
  *state_result_ds = xWS;
  *control_result_ds = uWS;
}

bool OpenSpaceTrajectoryOptimizer::GenerateDistanceApproachTraj(
    const Eigen::MatrixXd &xWS, const Eigen::MatrixXd &uWS,
    const std::vector<double> &XYbounds,
    const Eigen::MatrixXi &obstacles_edges_num,
    const Eigen::MatrixXd &f_driving_bound,
    const Eigen::MatrixXd &b_driving_bound, //行车隧道约束
    const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
    const Eigen::MatrixXd &last_time_u, const double init_v,
    Eigen::MatrixXd *state_result_ds, Eigen::MatrixXd *control_result_ds,
    Eigen::MatrixXd *time_result_ds) {
  size_t horizon = xWS.cols() - 1; // x范围
  Eigen::MatrixXd x0(4, 1);
  x0 << xWS(0, 0), xWS(1, 0), xWS(2, 0), init_v; //初始状态

  Eigen::MatrixXd xF(4, 1);
  xF << xWS(0, horizon), xWS(1, horizon), xWS(2, horizon),
      xWS(3, horizon); //终点状态

  // load vehicle configuration载入车辆参数
  VehicleParam vehicle_param_;
  double front_to_center = vehicle_param_.front_edge_to_center;
  double back_to_center = vehicle_param_.back_edge_to_center;
  double left_to_center = vehicle_param_.left_edge_to_center;
  double right_to_center = vehicle_param_.right_edge_to_center;
  Eigen::MatrixXd ego(4, 1);
  ego << front_to_center, right_to_center, back_to_center, left_to_center;

  // Get obstacle num障碍物数量
  size_t obstacles_num = obstacles_vertices_vec.size();

  // Get timestep delta t
  double ts_init = 1; // TA*/n  计算初始的ts

  //☆☆☆☆☆☆☆☆☆☆☆3.调用轨迹优化程序☆☆☆☆☆☆☆☆☆☆☆
  // Distance approach trajectory smoothing
  if (distance_approach_->Solve(
          x0, xF, last_time_u, horizon, ts_init, ego, xWS, uWS, XYbounds,
          obstacles_num, obstacles_edges_num, f_driving_bound, b_driving_bound,
          state_result_ds, control_result_ds, time_result_ds)) {
    std::cout << "Distance approach problem solved successfully!" << std::endl;
  } else {
    std::cout << "Distance approach problem solved failed!";
  }
  return true;
}

// TODO(Jinyun): deprecate the use of Eigen in trajectory smoothing
void OpenSpaceTrajectoryOptimizer::LoadHybridAstarResultInEigen(
    HybridAStartResult *result, Eigen::MatrixXd *xWS, Eigen::MatrixXd *uWS) {
  // load Warm Start result(horizon is timestep number minus one)
  size_t horizon = result->x.size() - 1;
  xWS->resize(4, horizon + 1); // 4行n列
  uWS->resize(2, horizon);     // 4行n-1列控制量少一个周期
  Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->x.data(), horizon + 1);
  Eigen::VectorXd y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->y.data(), horizon + 1);
  Eigen::VectorXd phi = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->phi.data(), horizon + 1);
  Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->v.data(), horizon + 1);
  Eigen::VectorXd steer = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->steer.data(), horizon);
  Eigen::VectorXd a =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(result->a.data(), horizon);
  xWS->row(0) = std::move(x);
  xWS->row(1) = std::move(y);
  xWS->row(2) = std::move(phi);
  xWS->row(3) = std::move(v);
  uWS->row(0) = std::move(steer);
  uWS->row(1) = std::move(a);
}

// TODO(Jinyun): tmp interface, will refactor
void OpenSpaceTrajectoryOptimizer::LoadResult(
    const DiscretizedTrajectory &discretized_trajectory,
    Eigen::MatrixXd *state_result_dc, Eigen::MatrixXd *control_result_dc,
    Eigen::MatrixXd *time_result_dc) {
  const size_t points_size = discretized_trajectory.size();
  // CHECK_GT(points_size, 1U);
  *state_result_dc = Eigen::MatrixXd::Zero(4, points_size);
  *control_result_dc = Eigen::MatrixXd::Zero(2, points_size - 1);
  *time_result_dc = Eigen::MatrixXd::Zero(1, points_size - 1);

  auto &state_result = *state_result_dc;
  for (size_t i = 0; i < points_size; ++i) {
    state_result(0, i) = discretized_trajectory[i].path_point().x();
    state_result(1, i) = discretized_trajectory[i].path_point().y();
    state_result(2, i) = discretized_trajectory[i].path_point().theta();
    state_result(3, i) = discretized_trajectory[i].v();
  }

  auto &control_result = *control_result_dc;
  auto &time_result = *time_result_dc;
  const double wheel_base = vehicle_param_.wheel_base;
  for (size_t i = 0; i + 1 < points_size; ++i) {
    control_result(0, i) =
        std::atan(discretized_trajectory[i].path_point().kappa() * wheel_base);
    control_result(1, i) = discretized_trajectory[i].a();
    time_result(0, i) = discretized_trajectory[i + 1].relative_time() -
                        discretized_trajectory[i].relative_time();
  }
}

void OpenSpaceTrajectoryOptimizer::GetOptimizedTrajectory(
    DiscretizedTrajectory *optimized_trajectory) {
  optimized_trajectory->clear();
  *optimized_trajectory = optimized_trajectory_;
}

void OpenSpaceTrajectoryOptimizer::GetCoarseTrajectory(
    DiscretizedTrajectory *coarse_trajectory) {
  // coarse_trajectory_->clear();
  *coarse_trajectory = coarse_trajectory_;
}

bool OpenSpaceTrajectoryOptimizer::GenerateCoarseTraj(
    const Eigen::MatrixXd &xWS, const Eigen::MatrixXd &uWS,
    const std::vector<double> &XYbounds,
    const Eigen::MatrixXi &obstacles_edges_num,
    const Eigen::MatrixXd &f_driving_bound,
    const Eigen::MatrixXd &b_driving_bound,
    const std::vector<std::vector<common::math::Vec2d>> &obstacles_vertices_vec,
    const Eigen::MatrixXd &last_time_u, const double init_v,
    Eigen::MatrixXd *state_result_ds, Eigen::MatrixXd *control_result_ds,
    Eigen::MatrixXd *time_result_ds) {

  UseWarmStartAsResult(xWS, uWS, state_result_ds, control_result_ds,
                       time_result_ds);

  LoadCoarseTrajectory((*state_result_ds), (*control_result_ds),
                       (*time_result_ds));
}

void OpenSpaceTrajectoryOptimizer::LoadCoarseTrajectory(
    const Eigen::MatrixXd &state_result, const Eigen::MatrixXd &control_result,
    const Eigen::MatrixXd &time_result) {
  coarse_trajectory_.clear();

  // Optimizer doesn't take end condition control state into consideration for
  // now
  size_t states_size = state_result.cols();
  size_t times_size = time_result.cols();
  size_t controls_size = control_result.cols();

  double relative_time = 0.0;
  double relative_s = 0.0;
  Vec2d last_path_point(state_result(0, 0), state_result(1, 0));
  for (size_t i = 0; i < states_size; ++i) {
    common::TrajectoryPoint point;
    point.set_x(state_result(0, i));
    point.set_y(state_result(1, i));
    point.set_theta(state_result(2, i));
    point.set_v(state_result(3, i));
    Vec2d cur_path_point(state_result(0, i), state_result(1, i));
    relative_s += cur_path_point.DistanceTo(last_path_point);
    point.set_s(relative_s);
    // TODO(Jinyun): Evaluate how to set end states control input
    if (i == controls_size) {
      point.set_steer(0.0);
      point.set_a(0.0);
    } else {
      point.set_steer(control_result(0, i));
      point.set_a(control_result(1, i));
    }

    if (i == 0) {
      point.set_relative_time(relative_time);
    } else {
      relative_time =
          config_.planner_open_space_config().warm_start_config.delta_t * i;
      point.set_relative_time(relative_time);
    }

    coarse_trajectory_.emplace_back(point);
    last_path_point = cur_path_point;
  }
}

Eigen::MatrixXd OpenSpaceTrajectoryOptimizer::GetFrontDrivingBound() {
  return f_bound_;
}
Eigen::MatrixXd OpenSpaceTrajectoryOptimizer::GetBackDrivingBound() {
  return r_bound_;
}

std::shared_ptr<ConstructDrivingCorridor>
OpenSpaceTrajectoryOptimizer::GetConstructCorridorPtr() {
  return construct_corridor_; //构建行车隧道
}
