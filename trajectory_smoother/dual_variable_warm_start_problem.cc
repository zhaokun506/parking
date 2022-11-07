/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/*
 * @file
 */

#include "trajectory_smoother/dual_variable_warm_start_problem.h"

#include <coin-or/IpIpoptApplication.hpp>
#include <coin-or/IpSolveStatistics.hpp>

// #include "modules/common/util/perf_util.h"
// #include "modules/planning/common/planning_gflags.h"

DualVariableWarmStartProblem::DualVariableWarmStartProblem(
    const PlannerOpenSpaceConfig &planner_open_space_config)
    : planner_open_space_config_(planner_open_space_config) {
  // planner_open_space_config_ = planner_open_space_config;
}

bool DualVariableWarmStartProblem::Solve(
    const size_t horizon, const double ts, const Eigen::MatrixXd &ego,
    size_t obstacles_num, const Eigen::MatrixXi &obstacles_edges_num,
    const Eigen::MatrixXd &obstacles_A, const Eigen::MatrixXd &obstacles_b,
    const Eigen::MatrixXd &xWS, Eigen::MatrixXd *l_warm_up,
    Eigen::MatrixXd *n_warm_up, Eigen::MatrixXd *s_warm_up) {
  // PERF_BLOCK_START()
  bool solver_flag = false;

  if (planner_open_space_config_.dual_variable_warm_start_config.qp_format ==
      "OSQP") {
    DualVariableWarmStartOSQPInterface ptop =
        DualVariableWarmStartOSQPInterface(
            horizon, ts, ego, obstacles_edges_num, obstacles_num, obstacles_A,
            obstacles_b, xWS, planner_open_space_config_);

    if (ptop.optimize()) {
      std::cout << "dual warm up done." << std::endl;
      ptop.get_optimization_results(l_warm_up, n_warm_up);

      //    PERF_BLOCK_END("DualVariableWarmStartSolving");
      solver_flag = true;
    } else {
      std::cout << "dual warm up fail." << std::endl;
      ptop.get_optimization_results(l_warm_up, n_warm_up);
      solver_flag = false;
    }
  }

  if (solver_flag == false) {
    // if solver fails during dual warm up, insert zeros instead
    for (int r = 0; r < l_warm_up->rows(); ++r) {
      for (int c = 0; c < l_warm_up->cols(); ++c) {
        (*l_warm_up)(r, c) = 0.0;
      }
    }

    for (int r = 0; r < n_warm_up->rows(); ++r) {
      for (int c = 0; c < n_warm_up->cols(); ++c) {
        (*n_warm_up)(r, c) = 0.0;
      }
    }
  }

  return true;
}
