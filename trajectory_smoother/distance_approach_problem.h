/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#pragma once
#include <iostream>
#include <vector>

#include <coin-or/IpIpoptApplication.hpp>
#include <coin-or/IpSolveStatistics.hpp>

#include "eigen3/Eigen/Dense"

//#include "modules/planning/proto/planning.pb.h"

//#include "modules/planning/common/planning_gflags.h"
#include "trajectory_smoother/distance_approach_ipopt_corridor_interface.h"

class DistanceApproachProblem {
public:
  explicit DistanceApproachProblem(
      const PlannerOpenSpaceConfig &planner_open_space_config);

  virtual ~DistanceApproachProblem() = default;
  //类的主函数
  bool Solve(const Eigen::MatrixXd &x0, const Eigen::MatrixXd &xF,
             const Eigen::MatrixXd &last_time_u, const size_t horizon,
             const double ds_init, const Eigen::MatrixXd &ego,
             const Eigen::MatrixXd &xWS, const Eigen::MatrixXd &uWS,
             const std::vector<double> &XYbounds, const size_t obstacles_num,
             const Eigen::MatrixXi &obstacles_edges_num,
             const Eigen::MatrixXd &f_driving_bound,
             const Eigen::MatrixXd &b_driving_bound,
             Eigen::MatrixXd *state_result, Eigen::MatrixXd *control_result,
             Eigen::MatrixXd *time_result);

private:
  PlannerOpenSpaceConfig planner_open_space_config_;
};
