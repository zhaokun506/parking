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
    : config_(config)
{
  // Load config加载配置
  config_ = config;
  // Initialize hybrid astar class pointer初始化混合A*指针，生成粗略初始轨迹
  warm_start_.reset(new HybridAStar(config.planner_open_space_config()));
  // 构建行车隧道
  construct_corridor_.reset(new HybridAStar(config.planner_open_space_config()));
  // Initialize distance approach trajectory smootherclass
  // pointer初始化距离接近优化算法指针
  distance_approach_.reset(
      new DistanceApproachProblem(config.planner_open_space_config()));
}

int OpenSpaceTrajectoryOptimizer::Plan(
    const MapPoint &start_pose; //起点
    const MapPoint &end_pose,   //终点
    const std::vector<double> &XYbounds,
    double rotate_angle, const common::math::Vec2d &translate_origin,
    const Eigen::MatrixXi &obstacles_edges_num,                                  // H表示中A和b矩阵维数所需的维数
    const Eigen::MatrixXd &obstacles_A, const Eigen::MatrixXd &obstacles_b,      //障碍物Ax＞b的线性不等式表示
    const std::vector<std::vector<common::math::Vec2d>> &obstacles_vertices_vec, //以逆时钟顺序存储障碍物顶点的向量
    double *time_latency);
{
  //判断输入数据是否正确
  if (XYbounds.empty() || obstacles_edges_num.cols() == 0 ||
      obstacles_A.cols() == 0 || obstacles_b.cols() == 0)
  {
    std::cout << "OpenSpaceTrajectoryOptimizer input data not ready";
    return 0;
  }

  const auto start_timestamp = std::chrono::system_clock::now();

  // init x, y, z would be rotated.
  double init_x = start_pose.x;
  double init_y = start_pose.y;
  double init_phi = start_pose.phi;
  double end_x = end_pos.x;
  double end_y = end_pos.y;
  double end_phi = end_pos.phi;

  // Result container for warm start (initial velocity is assumed to be 0 for
  // now)初始速度读假设为0
  HybridAStartResult result;
  //☆☆☆☆☆☆☆☆☆☆☆1.调用混合A*算法☆☆☆☆☆☆☆☆☆☆☆
  if (warm_start_->Plan(init_x, init_y, init_phi,
                        end_x, end_y, end_phi,
                        XYbounds, obstacles_vertices_vec,
                        &result))
  {
    std::cout << "State warm start problem solved successfully!";
  }
  else
  {
    std::cout << "State warm start problem failed to solve";
    return 0;
  }




  // Containers for distance approach trajectory smoothing problem
  //定义输入输出矩阵
  Eigen::MatrixXd xWS;
  Eigen::MatrixXd uWS;

  Eigen::MatrixXd state_result_ds;
  Eigen::MatrixXd control_result_ds;
  Eigen::MatrixXd time_result_ds;
  Eigen::MatrixXd l_warm_up; // l
  Eigen::MatrixXd n_warm_up; // n
  Eigen::MatrixXd dual_l_result_ds;
  Eigen::MatrixXd dual_n_result_ds;

  //不启用并行轨迹平滑
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
    time_result_ds         时间优化结果

    l_warm_up
    n_warm_up

    dual_l_result_ds
    dual_n_result_ds

  */
  //生成行车隧道

  construct_corridor_->construct(); //构建行车隧道

  

  if (!GenerateDistanceApproachTraj(
          xWS, uWS, XYbounds, obstacles_edges_num, obstacles_A, obstacles_b,
          obstacles_vertices_vec, last_time_u, init_v, &state_result_ds,
          &control_result_ds, &time_result_ds, &l_warm_up, &n_warm_up,
          &dual_l_result_ds, &dual_n_result_ds))
  {
    std::cout << "distance approach smoothing problem failed to solve" << std::endl;
    return 0;
  }

  LoadTrajectory(state_result_ds, control_result_ds, time_result_ds);

  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  std::cout << "open space trajectory smoother total time: "
            << diff.count() * 1000.0 << " ms.";
  *time_latency = diff.count() * 1000.0;

  return 1;
}

void OpenSpaceTrajectoryOptimizer::LoadTrajectory(
    const Eigen::MatrixXd &state_result, const Eigen::MatrixXd &control_result,
    const Eigen::MatrixXd &time_result)
{
  optimized_trajectory_.clear();

  // Optimizer doesn't take end condition control state into consideration for
  // now
  size_t states_size = state_result.cols();
  size_t times_size = time_result.cols();
  size_t controls_size = control_result.cols();
  CHECK_EQ(states_size, times_size + 1);
  CHECK_EQ(states_size, controls_size + 1);
  double relative_time = 0.0;
  double relative_s = 0.0;
  Vec2d last_path_point(state_result(0, 0), state_result(1, 0));
  for (size_t i = 0; i < states_size; ++i)
  {
    common::TrajectoryPoint point;
    point.mutable_path_point()->set_x(state_result(0, i));
    point.mutable_path_point()->set_y(state_result(1, i));
    point.mutable_path_point()->set_theta(state_result(2, i));
    point.set_v(state_result(3, i));
    Vec2d cur_path_point(state_result(0, i), state_result(1, i));
    relative_s += cur_path_point.DistanceTo(last_path_point);
    point.mutable_path_point()->set_s(relative_s);
    // TODO(Jinyun): Evaluate how to set end states control input
    if (i == controls_size)
    {
      point.set_steer(0.0);
      point.set_a(0.0);
    }
    else
    {
      point.set_steer(control_result(0, i));
      point.set_a(control_result(1, i));
    }

    if (i == 0)
    {
      point.set_relative_time(relative_time);
    }
    else
    {
      relative_time += time_result(0, i - 1);
      point.set_relative_time(relative_time);
    }

    optimized_trajectory_.emplace_back(point);
    last_path_point = cur_path_point;
  }
}

void OpenSpaceTrajectoryOptimizer::UseWarmStartAsResult(
    const Eigen::MatrixXd &xWS, const Eigen::MatrixXd &uWS,
    const Eigen::MatrixXd &l_warm_up, const Eigen::MatrixXd &n_warm_up,
    Eigen::MatrixXd *state_result_ds, Eigen::MatrixXd *control_result_ds,
    Eigen::MatrixXd *time_result_ds, Eigen::MatrixXd *dual_l_result_ds,
    Eigen::MatrixXd *dual_n_result_ds)
{
  AERROR << "Use warm start as trajectory output";

  *state_result_ds = xWS;
  *control_result_ds = uWS;
  *dual_l_result_ds = l_warm_up;
  *dual_n_result_ds = n_warm_up;

  size_t time_result_horizon = xWS.cols() - 1;
  *time_result_ds = Eigen::MatrixXd::Constant(
      1, time_result_horizon, config_.planner_open_space_config().delta_t());
}

bool OpenSpaceTrajectoryOptimizer::GenerateDistanceApproachTraj(
    const Eigen::MatrixXd &xWS, const Eigen::MatrixXd &uWS,
    const std::vector<double> &XYbounds,
    const Eigen::MatrixXi &obstacles_edges_num,
    const Eigen::MatrixXd &obstacles_A, const Eigen::MatrixXd &obstacles_b,
    const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
    const Eigen::MatrixXd &last_time_u, const double init_v,
    Eigen::MatrixXd *state_result_ds, Eigen::MatrixXd *control_result_ds,
    Eigen::MatrixXd *time_result_ds, Eigen::MatrixXd *l_warm_up,
    Eigen::MatrixXd *n_warm_up, Eigen::MatrixXd *dual_l_result_ds,
    Eigen::MatrixXd *dual_n_result_ds)
{
  size_t horizon = xWS.cols() - 1; // x范围
  Eigen::MatrixXd x0(4, 1);
  x0 << xWS(0, 0), xWS(1, 0), xWS(2, 0), init_v; //初始状态

  Eigen::MatrixXd xF(4, 1);
  xF << xWS(0, horizon), xWS(1, horizon), xWS(2, horizon), xWS(3, horizon); //终点状态

  // load vehicle configuration载入车辆参数
  const common::VehicleParam &vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double front_to_center = vehicle_param_.front_edge_to_center();
  double back_to_center = vehicle_param_.back_edge_to_center();
  double left_to_center = vehicle_param_.left_edge_to_center();
  double right_to_center = vehicle_param_.right_edge_to_center();
  Eigen::MatrixXd ego(4, 1);
  ego << front_to_center, right_to_center, back_to_center, left_to_center;

  // Get obstacle num障碍物数量
  size_t obstacles_num = obstacles_vertices_vec.size();

  // Get timestep delta t
  double ts = config_.planner_open_space_config().delta_t();

  // slack_warm_up, temp usage
  Eigen::MatrixXd s_warm_up = Eigen::MatrixXd::Zero(obstacles_num, horizon + 1);

 

  //☆☆☆☆☆☆☆☆☆☆☆3.调用轨迹优化程序☆☆☆☆☆☆☆☆☆☆☆
  // Distance approach trajectory smoothing
  if (distance_approach_->Solve(
          x0, xF, last_time_u, horizon, ts, ego, xWS, uWS, *l_warm_up,
          *n_warm_up, s_warm_up, XYbounds, obstacles_num, obstacles_edges_num,
          obstacles_A, obstacles_b, state_result_ds, control_result_ds,
          time_result_ds, dual_l_result_ds, dual_n_result_ds))
  {
    std::cout << "Distance approach problem solved successfully!";
  }
  else
  {
    std::cout << "Distance approach problem solved failed!";
  }
  return true;
}

// TODO(Jinyun): deprecate the use of Eigen in trajectory smoothing
void OpenSpaceTrajectoryOptimizer::LoadHybridAstarResultInEigen(
    HybridAStartResult *result, Eigen::MatrixXd *xWS, Eigen::MatrixXd *uWS)
{
  // load Warm Start result(horizon is timestep number minus one)
  size_t horizon = result->x.size() - 1;
  xWS->resize(4, horizon + 1);
  uWS->resize(2, horizon);
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

void OpenSpaceTrajectoryOptimizer::CombineTrajectories(
    const std::vector<Eigen::MatrixXd> &xWS_vec,
    const std::vector<Eigen::MatrixXd> &uWS_vec,
    const std::vector<Eigen::MatrixXd> &state_result_ds_vec,
    const std::vector<Eigen::MatrixXd> &control_result_ds_vec,
    const std::vector<Eigen::MatrixXd> &time_result_ds_vec,
    const std::vector<Eigen::MatrixXd> &l_warm_up_vec,
    const std::vector<Eigen::MatrixXd> &n_warm_up_vec,
    const std::vector<Eigen::MatrixXd> &dual_l_result_ds_vec,
    const std::vector<Eigen::MatrixXd> &dual_n_result_ds_vec,
    Eigen::MatrixXd *xWS, Eigen::MatrixXd *uWS,
    Eigen::MatrixXd *state_result_ds, Eigen::MatrixXd *control_result_ds,
    Eigen::MatrixXd *time_result_ds, Eigen::MatrixXd *l_warm_up,
    Eigen::MatrixXd *n_warm_up, Eigen::MatrixXd *dual_l_result_ds,
    Eigen::MatrixXd *dual_n_result_ds)
{
  // Repeated midway state point are not added
  size_t warm_start_state_size = 0;
  for (const auto &warm_start_state : xWS_vec)
  {
    warm_start_state_size += warm_start_state.cols();
  }
  warm_start_state_size -= xWS_vec.size() - 1;

  size_t warm_start_control_size = 0;
  for (const auto &warm_start_control : uWS_vec)
  {
    warm_start_control_size += warm_start_control.cols();
  }

  // Repeated midway state point are not added
  size_t smoothed_state_size = 0;
  for (const auto &smoothed_state : state_result_ds_vec)
  {
    smoothed_state_size += smoothed_state.cols();
  }
  smoothed_state_size -= state_result_ds_vec.size() - 1;

  size_t smoothed_control_size = 0;
  for (const auto &smoothed_control : control_result_ds_vec)
  {
    smoothed_control_size += smoothed_control.cols();
  }

  size_t time_size = 0;
  for (const auto &smoothed_time : time_result_ds_vec)
  {
    time_size += smoothed_time.cols();
  }

  size_t l_warm_start_size = 0;
  for (const auto &l_warm_start : l_warm_up_vec)
  {
    l_warm_start_size += l_warm_start.cols();
  }

  size_t n_warm_start_size = 0;
  for (const auto &n_warm_start : n_warm_up_vec)
  {
    n_warm_start_size += n_warm_start.cols();
  }

  size_t l_smoothed_size = 0;
  for (const auto &l_smoothed : dual_l_result_ds_vec)
  {
    l_smoothed_size += l_smoothed.cols();
  }

  size_t n_smoothed_size = 0;
  for (const auto &n_smoothed : dual_n_result_ds_vec)
  {
    n_smoothed_size += n_smoothed.cols();
  }

  Eigen::MatrixXd xWS_ =
      Eigen::MatrixXd::Zero(xWS_vec[0].rows(), warm_start_state_size);
  Eigen::MatrixXd uWS_ =
      Eigen::MatrixXd::Zero(uWS_vec[0].rows(), warm_start_control_size);
  Eigen::MatrixXd state_result_ds_ =
      Eigen::MatrixXd::Zero(state_result_ds_vec[0].rows(), smoothed_state_size);
  Eigen::MatrixXd control_result_ds_ = Eigen::MatrixXd::Zero(
      control_result_ds_vec[0].rows(), smoothed_control_size);
  Eigen::MatrixXd time_result_ds_ =
      Eigen::MatrixXd::Zero(time_result_ds_vec[0].rows(), time_size);
  Eigen::MatrixXd l_warm_up_ =
      Eigen::MatrixXd::Zero(l_warm_up_vec[0].rows(), l_warm_start_size);
  Eigen::MatrixXd n_warm_up_ =
      Eigen::MatrixXd::Zero(n_warm_up_vec[0].rows(), n_warm_start_size);
  Eigen::MatrixXd dual_l_result_ds_ =
      Eigen::MatrixXd::Zero(dual_l_result_ds_vec[0].rows(), l_smoothed_size);
  Eigen::MatrixXd dual_n_result_ds_ =
      Eigen::MatrixXd::Zero(dual_n_result_ds_vec[0].rows(), n_smoothed_size);

  size_t traj_size = xWS_vec.size();

  uint64_t counter = 0;
  for (size_t i = 0; i < traj_size; ++i)
  {
    // leave out the last repeated point so set column minus one
    uint64_t warm_start_state_cols = xWS_vec[i].cols() - 1;
    for (size_t j = 0; j < warm_start_state_cols; ++j)
    {
      xWS_.col(counter) = xWS_vec[i].col(j);
      ++counter;
    }
  }
  xWS_.col(counter) = xWS_vec.back().col(xWS_vec.back().cols() - 1);
  ++counter;
  CHECK_EQ(counter, warm_start_state_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i)
  {
    // leave out the last repeated point so set column minus one
    uint64_t warm_start_control_cols = uWS_vec[i].cols();
    for (size_t j = 0; j < warm_start_control_cols; ++j)
    {
      uWS_.col(counter) = uWS_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, warm_start_control_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i)
  {
    // leave out the last repeated point so set column minus one
    uint64_t smoothed_state_cols = state_result_ds_vec[i].cols() - 1;
    for (size_t j = 0; j < smoothed_state_cols; ++j)
    {
      state_result_ds_.col(counter) = state_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  state_result_ds_.col(counter) =
      state_result_ds_vec.back().col(state_result_ds_vec.back().cols() - 1);
  ++counter;
  CHECK_EQ(counter, smoothed_state_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i)
  {
    // leave out the last repeated point so set column minus one
    uint64_t smoothed_control_cols = control_result_ds_vec[i].cols();
    for (size_t j = 0; j < smoothed_control_cols; ++j)
    {
      control_result_ds_.col(counter) = control_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, smoothed_control_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i)
  {
    // leave out the last repeated point so set column minus one
    uint64_t time_cols = time_result_ds_vec[i].cols();
    for (size_t j = 0; j < time_cols; ++j)
    {
      time_result_ds_.col(counter) = time_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, time_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i)
  {
    // leave out the last repeated point so set column minus one
    uint64_t l_warm_up_cols = l_warm_up_vec[i].cols();
    for (size_t j = 0; j < l_warm_up_cols; ++j)
    {
      l_warm_up_.col(counter) = l_warm_up_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, l_warm_start_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i)
  {
    // leave out the last repeated point so set column minus one
    uint64_t n_warm_up_cols = n_warm_up_vec[i].cols();
    for (size_t j = 0; j < n_warm_up_cols; ++j)
    {
      n_warm_up_.col(counter) = n_warm_up_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, n_warm_start_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i)
  {
    // leave out the last repeated point so set column minus one
    uint64_t dual_l_result_ds_cols = dual_l_result_ds_vec[i].cols();
    for (size_t j = 0; j < dual_l_result_ds_cols; ++j)
    {
      dual_l_result_ds_.col(counter) = dual_l_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, l_smoothed_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i)
  {
    // leave out the last repeated point so set column minus one
    uint64_t dual_n_result_ds_cols = dual_n_result_ds_vec[i].cols();
    for (size_t j = 0; j < dual_n_result_ds_cols; ++j)
    {
      dual_n_result_ds_.col(counter) = dual_n_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, n_smoothed_size);

  *xWS = std::move(xWS_);
  *uWS = std::move(uWS_);
  *state_result_ds = std::move(state_result_ds_);
  *control_result_ds = std::move(control_result_ds_);
  *time_result_ds = std::move(time_result_ds_);
  *l_warm_up = std::move(l_warm_up_);
  *n_warm_up = std::move(n_warm_up_);
  *dual_l_result_ds = std::move(dual_l_result_ds_);
  *dual_n_result_ds = std::move(dual_n_result_ds_);
}

bool OpenSpaceTrajectoryOptimizer::GenerateDecoupledTraj(
    const Eigen::MatrixXd &xWS, const double init_a, const double init_v,
    const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
    Eigen::MatrixXd *state_result_dc, Eigen::MatrixXd *control_result_dc,
    Eigen::MatrixXd *time_result_dc)
{
  DiscretizedTrajectory smoothed_trajectory;
  if (!iterative_anchoring_smoother_->Smooth(
          xWS, init_a, init_v, obstacles_vertices_vec, &smoothed_trajectory))
  {
    return false;
  }

  LoadResult(smoothed_trajectory, state_result_dc, control_result_dc,
             time_result_dc);
  return true;
}

// TODO(Jinyun): tmp interface, will refactor
void OpenSpaceTrajectoryOptimizer::LoadResult(
    const DiscretizedTrajectory &discretized_trajectory,
    Eigen::MatrixXd *state_result_dc, Eigen::MatrixXd *control_result_dc,
    Eigen::MatrixXd *time_result_dc)
{
  const size_t points_size = discretized_trajectory.size();
  CHECK_GT(points_size, 1U);
  *state_result_dc = Eigen::MatrixXd::Zero(4, points_size);
  *control_result_dc = Eigen::MatrixXd::Zero(2, points_size - 1);
  *time_result_dc = Eigen::MatrixXd::Zero(1, points_size - 1);

  auto &state_result = *state_result_dc;
  for (size_t i = 0; i < points_size; ++i)
  {
    state_result(0, i) = discretized_trajectory[i].path_point().x();
    state_result(1, i) = discretized_trajectory[i].path_point().y();
    state_result(2, i) = discretized_trajectory[i].path_point().theta();
    state_result(3, i) = discretized_trajectory[i].v();
  }

  auto &control_result = *control_result_dc;
  auto &time_result = *time_result_dc;
  const double wheel_base = common::VehicleConfigHelper::Instance()
                                ->GetConfig()
                                .vehicle_param()
                                .wheel_base();
  for (size_t i = 0; i + 1 < points_size; ++i)
  {
    control_result(0, i) =
        std::atan(discretized_trajectory[i].path_point().kappa() * wheel_base);
    control_result(1, i) = discretized_trajectory[i].a();
    time_result(0, i) = discretized_trajectory[i + 1].relative_time() -
                        discretized_trajectory[i].relative_time();
  }
}
