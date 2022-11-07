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

#include "open_space_trajectory_optimizer/apollo_trajectory_optimizer.h"
#include "glog/logging.h"

#include <utility>

using common::math::Vec2d;

ApolloOpenSpaceTrajectoryOptimizer::ApolloOpenSpaceTrajectoryOptimizer(
    const OpenSpaceTrajectoryOptimizerConfig &config)
    : config_(config) {

  // Initialize hybrid astar class pointer
  warm_start_.reset(new HybridAStar(config_.planner_open_space_config()));

  // Initialize dual variable warm start class pointer
  dual_variable_warm_start_.reset(
      new DualVariableWarmStartProblem(config_.planner_open_space_config()));

  // Initialize distance approach trajectory smootherclass pointer
  distance_approach_.reset(
      new DistanceApproachApolloProblem(config_.planner_open_space_config()));

  // Initialize iterative anchoring smoother config class pointer
  // iterative_anchoring_smoother_.reset(
  //     new IterativeAnchoringSmoother(config_.planner_open_space_config()));
}

bool ApolloOpenSpaceTrajectoryOptimizer::Plan(
    const std::vector<common::TrajectoryPoint> &stitching_trajectory,
    const std::vector<double> &end_pose, const std::vector<double> &XYbounds,
    double rotate_angle, const Vec2d &translate_origin,
    const Eigen::MatrixXi &obstacles_edges_num,
    const Eigen::MatrixXd &obstacles_A, const Eigen::MatrixXd &obstacles_b,
    const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
    double *time_latency) {
  if (XYbounds.empty() || end_pose.empty() || obstacles_edges_num.cols() == 0 ||
      obstacles_A.cols() == 0 || obstacles_b.cols() == 0) {
    std::cout << "ApolloOpenSpaceTrajectoryOptimizer input data not ready"
              << std::endl;
    return 0;
  }

  // Generate Stop trajectory if init point close to destination
  if (IsInitPointNearDestination(stitching_trajectory.back(), end_pose,
                                 rotate_angle, translate_origin)) {
    std::cout << "Planning init point is close to destination, skip new "
                 "trajectory generation";
    return 0;
  }

  const auto start_timestamp = std::chrono::system_clock::now();

  // Initiate initial states
  stitching_trajectory_ = stitching_trajectory;

  // Init trajectory point is the stitching point from last trajectory
  const common::TrajectoryPoint trajectory_stitching_point =
      stitching_trajectory.back();

  // init x, y, z would be rotated.
  double init_x = trajectory_stitching_point.path_point().x();
  double init_y = trajectory_stitching_point.path_point().y();
  double init_phi = trajectory_stitching_point.path_point().theta();
  //   std::cout << "origin x: " << std::setprecision(9) <<
  //   translate_origin.x(); std::cout << "origin y: " << std::setprecision(9)
  //   << translate_origin.y(); std::cout << "init_x: " << std::setprecision(9)
  //   << init_x; std::cout << "init_y: " << std::setprecision(9) << init_y;

  // Rotate and scale the state
  PathPointNormalizing(rotate_angle, translate_origin, &init_x, &init_y,
                       &init_phi);

  // Result container for warm start (initial velocity is assumed to be 0 for
  // now)
  HybridAStartResult result;

  if (warm_start_->Plan(init_x, init_y, init_phi, end_pose[0], end_pose[1],
                        end_pose[2], XYbounds, obstacles_vertices_vec,
                        &result)) {
    std::cout << "State warm start problem solved successfully!";
  } else {
    std::cout << "State warm start problem failed to solve";
    return 0;
  }

  // Containers for distance approach trajectory smoothing problem
  Eigen::MatrixXd xWS;
  Eigen::MatrixXd uWS;
  Eigen::MatrixXd state_result_ds;
  Eigen::MatrixXd control_result_ds;
  Eigen::MatrixXd time_result_ds;
  Eigen::MatrixXd l_warm_up;
  Eigen::MatrixXd n_warm_up;
  Eigen::MatrixXd dual_l_result_ds;
  Eigen::MatrixXd dual_n_result_ds;

  bool FLAGS_enable_parallel_trajectory_smoothing = false;
  if (FLAGS_enable_parallel_trajectory_smoothing) {
    std::vector<HybridAStartResult> partition_trajectories;
    if (!warm_start_->TrajectoryPartition(result, &partition_trajectories)) {
      return 0;
    }
    size_t size = partition_trajectories.size();
    std::vector<Eigen::MatrixXd> xWS_vec;
    std::vector<Eigen::MatrixXd> uWS_vec;
    std::vector<Eigen::MatrixXd> state_result_ds_vec;
    std::vector<Eigen::MatrixXd> control_result_ds_vec;
    std::vector<Eigen::MatrixXd> time_result_ds_vec;
    std::vector<Eigen::MatrixXd> l_warm_up_vec;
    std::vector<Eigen::MatrixXd> n_warm_up_vec;
    std::vector<Eigen::MatrixXd> dual_l_result_ds_vec;
    std::vector<Eigen::MatrixXd> dual_n_result_ds_vec;
    xWS_vec.resize(size);
    uWS_vec.resize(size);
    state_result_ds_vec.resize(size);
    control_result_ds_vec.resize(size);
    time_result_ds_vec.resize(size);
    l_warm_up_vec.resize(size);
    n_warm_up_vec.resize(size);
    dual_l_result_ds_vec.resize(size);
    dual_n_result_ds_vec.resize(size);

    // In for loop
    std::cout << "Trajectories size in smoother is " << size;
    for (size_t i = 0; i < size; ++i) {
      LoadHybridAstarResultInEigen(&partition_trajectories[i], &xWS_vec[i],
                                   &uWS_vec[i]);
      // checking initial and ending points
      //   if (config_.planner_open_space_config()
      //           .enable_check_parallel_trajectory) {
      //     AINFO << "trajectory id: " << i;
      //     AINFO << "trajectory partitioned size: " << xWS_vec[i].cols();
      //     AINFO << "initial point: " << xWS_vec[i].col(0).transpose();
      //     AINFO << "ending point: "
      //           << xWS_vec[i].col(xWS_vec[i].cols() - 1).transpose();
      //   }

      Eigen::MatrixXd last_time_u(2, 1);
      double init_v = 0.0;
      // Stitching point control and velocity is set for first piece of
      // trajectories. In the next ones, control and velocity are assumed to be
      // zero as the next trajectories always start from vehicle static state
      if (i == 0) {
        const double init_steer = trajectory_stitching_point.steer();
        const double init_a = trajectory_stitching_point.a();
        last_time_u << init_steer, init_a;
        init_v = trajectory_stitching_point.v();
      } else {
        last_time_u << 0.0, 0.0;
        init_v = 0.0;
      }
      // TODO(Jinyun): Further testing
      const auto smoother_start_timestamp = std::chrono::system_clock::now();
      bool FLAGS_use_iterative_anchoring_smoother = false;
      if (FLAGS_use_iterative_anchoring_smoother) {
        if (!GenerateDecoupledTraj(
                xWS_vec[i], last_time_u(1, 0), init_v, obstacles_vertices_vec,
                &state_result_ds_vec[i], &control_result_ds_vec[i],
                &time_result_ds_vec[i])) {
          std::cout << "Smoother fail at " << i << "th trajectory";
          std::cout << i << "th trajectory size is " << xWS_vec[i].cols();
          return 0;
        }
      } else {
        const double start_system_timestamp =
            std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
        if (!GenerateDistanceApproachTraj(
                xWS_vec[i], uWS_vec[i], XYbounds, obstacles_edges_num,
                obstacles_A, obstacles_b, obstacles_vertices_vec, last_time_u,
                init_v, &state_result_ds_vec[i], &control_result_ds_vec[i],
                &time_result_ds_vec[i], &l_warm_up_vec[i], &n_warm_up_vec[i],
                &dual_l_result_ds_vec[i], &dual_n_result_ds_vec[i])) {
          std::cout << "Smoother fail at " << i
                    << "th trajectory with index starts from 0";
          std::cout << i << "th trajectory size is " << xWS_vec[i].cols();
          std::cout << "State matrix: " << xWS_vec[i];
          std::cout << "Control matrix: " << uWS_vec[i];
          return 0;
        }
        const auto end_system_timestamp =
            std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
        const auto time_diff_ms =
            (end_system_timestamp - start_system_timestamp) * 1000;
        std::cout << "total planning time spend: " << time_diff_ms << " ms.";
        std::cout << i << "th trajectory size is " << xWS_vec[i].cols();
        std::cout << "average time spend: " << time_diff_ms / xWS_vec[i].cols()
                  << " ms per point.";
        std::cout << "average time spend after smooth: "
                  << time_diff_ms / state_result_ds_vec[i].cols()
                  << " ms per point.";
        std::cout << i << "th smoothed trajectory size is "
                  << state_result_ds_vec[i].cols();
      }
      const auto smoother_end_timestamp = std::chrono::system_clock::now();
      std::chrono::duration<double> smoother_diff =
          smoother_end_timestamp - smoother_start_timestamp;
      std::cout << "Open space trajectory smoothing total time: "
                << smoother_diff.count() * 1000.0 << " ms at the " << i
                << "th trajectory.";
      std::cout << "The " << i << "th trajectory pre-smoothing size is "
                << xWS_vec[i].cols() << "; post-smoothing size is "
                << state_result_ds_vec[i].cols();
    }

    // Retrive the trajectory in one piece
    CombineTrajectories(xWS_vec, uWS_vec, state_result_ds_vec,
                        control_result_ds_vec, time_result_ds_vec,
                        l_warm_up_vec, n_warm_up_vec, dual_l_result_ds_vec,
                        dual_n_result_ds_vec, &xWS, &uWS, &state_result_ds,
                        &control_result_ds, &time_result_ds, &l_warm_up,
                        &n_warm_up, &dual_l_result_ds, &dual_n_result_ds);

  } else {
    LoadHybridAstarResultInEigen(&result, &xWS, &uWS);

    const double init_steer = trajectory_stitching_point.steer();
    const double init_a = trajectory_stitching_point.a();
    Eigen::MatrixXd last_time_u(2, 1);
    last_time_u << init_steer, init_a;

    const double init_v = trajectory_stitching_point.v();
    bool FLAGS_generate_coarse_trajectory_smoothing = true;
    if (FLAGS_generate_coarse_trajectory_smoothing) {
      GenerateCoarseTraj(xWS, uWS, XYbounds, obstacles_edges_num, obstacles_A,
                         obstacles_b, obstacles_vertices_vec, last_time_u,
                         init_v, &state_result_ds, &control_result_ds,
                         &time_result_ds, &l_warm_up, &n_warm_up,
                         &dual_l_result_ds, &dual_n_result_ds);
      LoadCoarseTrajectory(state_result_ds, control_result_ds, time_result_ds);
    }

    if (!GenerateDistanceApproachTraj(
            xWS, uWS, XYbounds, obstacles_edges_num, obstacles_A, obstacles_b,
            obstacles_vertices_vec, last_time_u, init_v, &state_result_ds,
            &control_result_ds, &time_result_ds, &l_warm_up, &n_warm_up,
            &dual_l_result_ds, &dual_n_result_ds)) {
      return 0;
    }
  }

  // record debug info
  bool FLAGS_enable_record_debug = false;
  if (FLAGS_enable_record_debug) {
    // open_space_debug_.Clear();
    // RecordDebugInfo(trajectory_stitching_point, translate_origin,
    // rotate_angle,
    //                 end_pose, xWS, uWS, l_warm_up, n_warm_up,
    //                 dual_l_result_ds, dual_n_result_ds, state_result_ds,
    //                 control_result_ds, time_result_ds, XYbounds,
    //                 obstacles_vertices_vec);
  }

  // rescale the states to the world frame
  size_t state_size = state_result_ds.cols();
  for (size_t i = 0; i < state_size; ++i) {
    PathPointDeNormalizing(rotate_angle, translate_origin,
                           &(state_result_ds(0, i)), &(state_result_ds(1, i)),
                           &(state_result_ds(2, i)));
  }

  LoadTrajectory(state_result_ds, control_result_ds, time_result_ds);

  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  std::cout << "open space trajectory smoother total time: "
            << diff.count() * 1000.0 << " ms.";
  *time_latency = diff.count() * 1000.0;

  return 1;
}

bool ApolloOpenSpaceTrajectoryOptimizer::IsInitPointNearDestination(
    const common::TrajectoryPoint &planning_init_point,
    const std::vector<double> &end_pose, double rotate_angle,
    const Vec2d &translate_origin) {
  CHECK_EQ(end_pose.size(), 4U);
  Vec2d end_pose_to_world_frame = Vec2d(end_pose[0], end_pose[1]);

  end_pose_to_world_frame.SelfRotate(rotate_angle);
  end_pose_to_world_frame += translate_origin;

  const common::PathPoint path_point = planning_init_point.path_point();
  double distance_to_init_point =
      std::sqrt((path_point.x() - end_pose_to_world_frame.x()) *
                    (path_point.x() - end_pose_to_world_frame.x()) +
                (path_point.y() - end_pose_to_world_frame.y()) *
                    (path_point.y() - end_pose_to_world_frame.y()));

  if (distance_to_init_point <
      config_.planner_open_space_config().is_near_destination_threshold) {
    return true;
  }
  return false;
}

void ApolloOpenSpaceTrajectoryOptimizer::PathPointNormalizing(
    double rotate_angle, const Vec2d &translate_origin, double *x, double *y,
    double *phi) {
  *x -= translate_origin.x();
  *y -= translate_origin.y();
  double tmp_x = *x;
  *x = (*x) * std::cos(-rotate_angle) - (*y) * std::sin(-rotate_angle);
  *y = tmp_x * std::sin(-rotate_angle) + (*y) * std::cos(-rotate_angle);
  *phi = common::math::NormalizeAngle(*phi - rotate_angle);
}

void ApolloOpenSpaceTrajectoryOptimizer::PathPointDeNormalizing(
    double rotate_angle, const Vec2d &translate_origin, double *x, double *y,
    double *phi) {
  double tmp_x = *x;
  *x = (*x) * std::cos(rotate_angle) - (*y) * std::sin(rotate_angle);
  *y = tmp_x * std::sin(rotate_angle) + (*y) * std::cos(rotate_angle);
  *x += translate_origin.x();
  *y += translate_origin.y();
  *phi = common::math::NormalizeAngle(*phi + rotate_angle);
}

void ApolloOpenSpaceTrajectoryOptimizer::LoadTrajectory(
    const Eigen::MatrixXd &state_result, const Eigen::MatrixXd &control_result,
    const Eigen::MatrixXd &time_result) {
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
  for (size_t i = 0; i < states_size; ++i) {
    common::TrajectoryPoint point;
    point.mutable_path_point()->set_x(state_result(0, i));
    point.mutable_path_point()->set_y(state_result(1, i));
    point.mutable_path_point()->set_theta(state_result(2, i));
    point.set_v(state_result(3, i));
    Vec2d cur_path_point(state_result(0, i), state_result(1, i));
    relative_s += cur_path_point.DistanceTo(last_path_point);
    point.mutable_path_point()->set_s(relative_s);
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
      relative_time += time_result(0, i - 1);
      point.set_relative_time(relative_time);
    }

    optimized_trajectory_.emplace_back(point);
    last_path_point = cur_path_point;
  }
}

void ApolloOpenSpaceTrajectoryOptimizer::UseWarmStartAsResult(
    const Eigen::MatrixXd &xWS, const Eigen::MatrixXd &uWS,
    const Eigen::MatrixXd &l_warm_up, const Eigen::MatrixXd &n_warm_up,
    Eigen::MatrixXd *state_result_ds, Eigen::MatrixXd *control_result_ds,
    Eigen::MatrixXd *time_result_ds, Eigen::MatrixXd *dual_l_result_ds,
    Eigen::MatrixXd *dual_n_result_ds) {
  std::cout << "Use warm start as trajectory output" << std::endl;

  *state_result_ds = xWS;
  *control_result_ds = uWS;
  *dual_l_result_ds = l_warm_up;
  *dual_n_result_ds = n_warm_up;

  size_t time_result_horizon = xWS.cols() - 1;
  *time_result_ds = Eigen::MatrixXd::Constant(
      1, time_result_horizon, config_.planner_open_space_config().delta_t);
}

bool ApolloOpenSpaceTrajectoryOptimizer::GenerateDistanceApproachTraj(
    const Eigen::MatrixXd &xWS, const Eigen::MatrixXd &uWS,
    const std::vector<double> &XYbounds,
    const Eigen::MatrixXi &obstacles_edges_num,
    const Eigen::MatrixXd &obstacles_A, const Eigen::MatrixXd &obstacles_b,
    const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
    const Eigen::MatrixXd &last_time_u, const double init_v,
    Eigen::MatrixXd *state_result_ds, Eigen::MatrixXd *control_result_ds,
    Eigen::MatrixXd *time_result_ds, Eigen::MatrixXd *l_warm_up,
    Eigen::MatrixXd *n_warm_up, Eigen::MatrixXd *dual_l_result_ds,
    Eigen::MatrixXd *dual_n_result_ds) {
  size_t horizon = xWS.cols() - 1;
  Eigen::MatrixXd x0(4, 1);
  x0 << xWS(0, 0), xWS(1, 0), xWS(2, 0), init_v;

  Eigen::MatrixXd xF(4, 1);
  xF << xWS(0, horizon), xWS(1, horizon), xWS(2, horizon), xWS(3, horizon);

  // load vehicle configuration
  double front_to_center = vehicle_param_.front_edge_to_center;
  double back_to_center = vehicle_param_.back_edge_to_center;
  double left_to_center = vehicle_param_.left_edge_to_center;
  double right_to_center = vehicle_param_.right_edge_to_center;
  Eigen::MatrixXd ego(4, 1);
  ego << front_to_center, right_to_center, back_to_center, left_to_center;

  // Get obstacle num
  size_t obstacles_num = obstacles_vertices_vec.size();

  // Get timestep delta t
  double ts = config_.planner_open_space_config().delta_t;

  // slack_warm_up, temp usage
  Eigen::MatrixXd s_warm_up = Eigen::MatrixXd::Zero(obstacles_num, horizon + 1);

  // Dual variable warm start for distance approach problem//对偶变量初始化
  bool FLAGS_use_dual_variable_warm_start = true;
  if (FLAGS_use_dual_variable_warm_start) {
    if (dual_variable_warm_start_->Solve(
            horizon, ts, ego, obstacles_num, obstacles_edges_num, obstacles_A,
            obstacles_b, xWS, l_warm_up, n_warm_up, &s_warm_up)) {
      std::cout << "Dual variable problem solved successfully!";
    } else {
      std::cout << "Dual variable problem failed to solve";
      return false;
    }
  } else {
    // lamda 障碍物超平面约束
    *l_warm_up =
        0.5 * Eigen::MatrixXd::Ones(obstacles_edges_num.sum(), horizon + 1);
    // niu自车超平面
    *n_warm_up = 0.5 * Eigen::MatrixXd::Ones(4 * obstacles_num, horizon + 1);
  }

  // Distance approach trajectory smoothing
  if (distance_approach_->Solve(
          x0, xF, last_time_u, horizon, ts, ego, xWS, uWS, *l_warm_up,
          *n_warm_up, s_warm_up, XYbounds, obstacles_num, obstacles_edges_num,
          obstacles_A, obstacles_b, state_result_ds, control_result_ds,
          time_result_ds, dual_l_result_ds, dual_n_result_ds)) {
    std::cout << "Distance approach problem solved successfully!" << std::endl;
  } else {
    std::cout << "Distance approach problem failed to solve" << std::endl;
    bool FLAGS_enable_smoother_failsafe = true;
    if (FLAGS_enable_smoother_failsafe) {
      UseWarmStartAsResult(xWS, uWS, *l_warm_up, *n_warm_up, state_result_ds,
                           control_result_ds, time_result_ds, dual_l_result_ds,
                           dual_n_result_ds);
    } else {
      //对路径进行简单的平滑处理输出
      return false;
    }
  }
  return true;
}

// TODO(Jinyun): deprecate the use of Eigen in trajectory smoothing
void ApolloOpenSpaceTrajectoryOptimizer::LoadHybridAstarResultInEigen(
    HybridAStartResult *result, Eigen::MatrixXd *xWS, Eigen::MatrixXd *uWS) {
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

void ApolloOpenSpaceTrajectoryOptimizer::CombineTrajectories(
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
    Eigen::MatrixXd *dual_n_result_ds) {
  // Repeated midway state point are not added
  size_t warm_start_state_size = 0;
  for (const auto &warm_start_state : xWS_vec) {
    warm_start_state_size += warm_start_state.cols();
  }
  warm_start_state_size -= xWS_vec.size() - 1;

  size_t warm_start_control_size = 0;
  for (const auto &warm_start_control : uWS_vec) {
    warm_start_control_size += warm_start_control.cols();
  }

  // Repeated midway state point are not added
  size_t smoothed_state_size = 0;
  for (const auto &smoothed_state : state_result_ds_vec) {
    smoothed_state_size += smoothed_state.cols();
  }
  smoothed_state_size -= state_result_ds_vec.size() - 1;

  size_t smoothed_control_size = 0;
  for (const auto &smoothed_control : control_result_ds_vec) {
    smoothed_control_size += smoothed_control.cols();
  }

  size_t time_size = 0;
  for (const auto &smoothed_time : time_result_ds_vec) {
    time_size += smoothed_time.cols();
  }

  size_t l_warm_start_size = 0;
  for (const auto &l_warm_start : l_warm_up_vec) {
    l_warm_start_size += l_warm_start.cols();
  }

  size_t n_warm_start_size = 0;
  for (const auto &n_warm_start : n_warm_up_vec) {
    n_warm_start_size += n_warm_start.cols();
  }

  size_t l_smoothed_size = 0;
  for (const auto &l_smoothed : dual_l_result_ds_vec) {
    l_smoothed_size += l_smoothed.cols();
  }

  size_t n_smoothed_size = 0;
  for (const auto &n_smoothed : dual_n_result_ds_vec) {
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
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t warm_start_state_cols = xWS_vec[i].cols() - 1;
    for (size_t j = 0; j < warm_start_state_cols; ++j) {
      xWS_.col(counter) = xWS_vec[i].col(j);
      ++counter;
    }
  }
  xWS_.col(counter) = xWS_vec.back().col(xWS_vec.back().cols() - 1);
  ++counter;
  CHECK_EQ(counter, warm_start_state_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t warm_start_control_cols = uWS_vec[i].cols();
    for (size_t j = 0; j < warm_start_control_cols; ++j) {
      uWS_.col(counter) = uWS_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, warm_start_control_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t smoothed_state_cols = state_result_ds_vec[i].cols() - 1;
    for (size_t j = 0; j < smoothed_state_cols; ++j) {
      state_result_ds_.col(counter) = state_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  state_result_ds_.col(counter) =
      state_result_ds_vec.back().col(state_result_ds_vec.back().cols() - 1);
  ++counter;
  CHECK_EQ(counter, smoothed_state_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t smoothed_control_cols = control_result_ds_vec[i].cols();
    for (size_t j = 0; j < smoothed_control_cols; ++j) {
      control_result_ds_.col(counter) = control_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, smoothed_control_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t time_cols = time_result_ds_vec[i].cols();
    for (size_t j = 0; j < time_cols; ++j) {
      time_result_ds_.col(counter) = time_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, time_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t l_warm_up_cols = l_warm_up_vec[i].cols();
    for (size_t j = 0; j < l_warm_up_cols; ++j) {
      l_warm_up_.col(counter) = l_warm_up_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, l_warm_start_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t n_warm_up_cols = n_warm_up_vec[i].cols();
    for (size_t j = 0; j < n_warm_up_cols; ++j) {
      n_warm_up_.col(counter) = n_warm_up_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, n_warm_start_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t dual_l_result_ds_cols = dual_l_result_ds_vec[i].cols();
    for (size_t j = 0; j < dual_l_result_ds_cols; ++j) {
      dual_l_result_ds_.col(counter) = dual_l_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, l_smoothed_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t dual_n_result_ds_cols = dual_n_result_ds_vec[i].cols();
    for (size_t j = 0; j < dual_n_result_ds_cols; ++j) {
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

bool ApolloOpenSpaceTrajectoryOptimizer::GenerateDecoupledTraj(
    const Eigen::MatrixXd &xWS, const double init_a, const double init_v,
    const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
    Eigen::MatrixXd *state_result_dc, Eigen::MatrixXd *control_result_dc,
    Eigen::MatrixXd *time_result_dc) {
  // DiscretizedTrajectory smoothed_trajectory;
  // if (!iterative_anchoring_smoother_->Smooth(
  //         xWS, init_a, init_v, obstacles_vertices_vec, &smoothed_trajectory))
  //         {
  //   return false;
  // }

  // LoadResult(smoothed_trajectory, state_result_dc, control_result_dc,
  //            time_result_dc);
  return true;
}

// TODO(Jinyun): tmp interface, will refactor
void ApolloOpenSpaceTrajectoryOptimizer::LoadResult(
    const DiscretizedTrajectory &discretized_trajectory,
    Eigen::MatrixXd *state_result_dc, Eigen::MatrixXd *control_result_dc,
    Eigen::MatrixXd *time_result_dc) {
  const size_t points_size = discretized_trajectory.size();
  CHECK_GT(points_size, 1U);
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
  VehicleParam vehicle_param;
  const double wheel_base = vehicle_param.wheel_base;
  for (size_t i = 0; i + 1 < points_size; ++i) {
    control_result(0, i) =
        std::atan(discretized_trajectory[i].path_point().kappa() * wheel_base);
    control_result(1, i) = discretized_trajectory[i].a();
    time_result(0, i) = discretized_trajectory[i + 1].relative_time() -
                        discretized_trajectory[i].relative_time();
  }
}

bool ApolloOpenSpaceTrajectoryOptimizer::GenerateCoarseTraj(
    const Eigen::MatrixXd &xWS, const Eigen::MatrixXd &uWS,
    const std::vector<double> &XYbounds,
    const Eigen::MatrixXi &obstacles_edges_num,
    const Eigen::MatrixXd &obstacles_A, const Eigen::MatrixXd &obstacles_b,
    const std::vector<std::vector<common::math::Vec2d>> &obstacles_vertices_vec,
    const Eigen::MatrixXd &last_time_u, const double init_v,
    Eigen::MatrixXd *state_result_ds, Eigen::MatrixXd *control_result_ds,
    Eigen::MatrixXd *time_result_ds, Eigen::MatrixXd *l_warm_up,
    Eigen::MatrixXd *n_warm_up, Eigen::MatrixXd *dual_l_result_ds,
    Eigen::MatrixXd *dual_n_result_ds) {

  UseWarmStartAsResult(xWS, uWS, *l_warm_up, *n_warm_up, state_result_ds,
                       control_result_ds, time_result_ds, dual_l_result_ds,
                       dual_n_result_ds);
}

void ApolloOpenSpaceTrajectoryOptimizer::LoadCoarseTrajectory(
    const Eigen::MatrixXd &state_result, const Eigen::MatrixXd &control_result,
    const Eigen::MatrixXd &time_result) {
  coarse_trajectory_.clear();

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
  for (size_t i = 0; i < states_size; ++i) {
    common::TrajectoryPoint point;
    point.mutable_path_point()->set_x(state_result(0, i));
    point.mutable_path_point()->set_y(state_result(1, i));
    point.mutable_path_point()->set_theta(state_result(2, i));
    point.set_v(state_result(3, i));
    Vec2d cur_path_point(state_result(0, i), state_result(1, i));
    relative_s += cur_path_point.DistanceTo(last_path_point);
    point.mutable_path_point()->set_s(relative_s);
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
      relative_time += time_result(0, i - 1);
      point.set_relative_time(relative_time);
    }

    coarse_trajectory_.emplace_back(point);
    last_path_point = cur_path_point;
  }
}
