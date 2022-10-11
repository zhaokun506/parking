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

#include "trajectory_smoother/distance_approach_problem.h"

#include <string>
#include <unordered_map>

//#include "modules/common/util/perf_util.h"

DistanceApproachProblem::DistanceApproachProblem(
    const PlannerOpenSpaceConfig &planner_open_space_config)
    : planner_open_space_config_(planner_open_space_config) {}

bool DistanceApproachProblem::Solve(
    //初始化状态，终点状态
    const Eigen::MatrixXd &x0, const Eigen::MatrixXd &xF,
    //控制末状态，离散个数，时间间隔
    const Eigen::MatrixXd &last_time_u, const size_t horizon,
    const double ds_init,
    //自车参数，状态量初始值
    const Eigen::MatrixXd &ego, const Eigen::MatrixXd &xWS,
    //控制量初始值，λ初始值
    const Eigen::MatrixXd &uWS,
    // xy边界，障碍物数量
    const std::vector<double> &XYbounds, const size_t obstacles_num,
    //障碍物边的个数
    const Eigen::MatrixXi &obstacles_edges_num,
    // Ab障碍物
    const Eigen::MatrixXd &f_driving_bound,
    const Eigen::MatrixXd &b_driving_bound, Eigen::MatrixXd *state_result,
    Eigen::MatrixXd *control_result, //返回的优化结果
    Eigen::MatrixXd *time_result) {
  // TODO(QiL) : evaluate whether need to new it everytime
  // PERF_BLOCK_START();
  //定义父类指针，通过判断指向子类对象，调用不同的优化程序
  DistanceApproachInterface *ptop = nullptr; //接口类指针

  if (planner_open_space_config_.distance_approach_config
          .distance_approach_mode ==
      "DISTANCE_APPROACH_CORRIDOR_IPOPT") { //默认模式
  } else if (planner_open_space_config_.distance_approach_config
                 .distance_approach_mode ==
             "DISTANCE_APPROACH_CORRIDOR_IPOPT") {
    ptop = new DistanceApproachIPOPTCorridorInterface(
        horizon, ds_init, ego, xWS, uWS, x0, xF, last_time_u, XYbounds,
        obstacles_edges_num, obstacles_num, f_driving_bound, b_driving_bound,
        planner_open_space_config_);
  }

  Ipopt::SmartPtr<Ipopt::TNLP> problem = ptop; //将指针传递给problem IPOPT

  // Create an instance of the IpoptApplication//实例化一个IPOPT对象
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  //对优化器进行设置
  app->Options()->SetIntegerValue(
      "print_level", planner_open_space_config_.distance_approach_config
                         .ipopt_config.ipopt_print_level);
  app->Options()->SetIntegerValue(
      "mumps_mem_percent", planner_open_space_config_.distance_approach_config
                               .ipopt_config.mumps_mem_percent);
  app->Options()->SetNumericValue(
      "mumps_pivtol", planner_open_space_config_.distance_approach_config
                          .ipopt_config.mumps_pivtol);
  app->Options()->SetIntegerValue(
      "max_iter", planner_open_space_config_.distance_approach_config
                      .ipopt_config.ipopt_max_iter);
  app->Options()->SetNumericValue(
      "tol", planner_open_space_config_.distance_approach_config.ipopt_config
                 .ipopt_tol);
  app->Options()->SetNumericValue(
      "acceptable_constr_viol_tol",
      planner_open_space_config_.distance_approach_config.ipopt_config
          .ipopt_acceptable_constr_viol_tol);
  app->Options()->SetNumericValue(
      "min_hessian_perturbation",
      planner_open_space_config_.distance_approach_config.ipopt_config
          .ipopt_min_hessian_perturbation);
  app->Options()->SetNumericValue(
      "jacobian_regularization_value",
      planner_open_space_config_.distance_approach_config.ipopt_config
          .ipopt_jacobian_regularization_value);
  app->Options()->SetStringValue(
      "print_timing_statistics",
      planner_open_space_config_.distance_approach_config.ipopt_config
          .ipopt_print_timing_statistics);
  app->Options()->SetStringValue(
      "alpha_for_y", planner_open_space_config_.distance_approach_config
                         .ipopt_config.ipopt_alpha_for_y);
  app->Options()->SetStringValue(
      "recalc_y", planner_open_space_config_.distance_approach_config
                      .ipopt_config.ipopt_recalc_y);
  app->Options()->SetNumericValue(
      "mu_init", planner_open_space_config_.distance_approach_config
                     .ipopt_config.ipopt_mu_init);

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    std::cout << "*** Distance Approach problem error during initialization!";
    return false;
  }

  status = app->OptimizeTNLP(problem); //将问题描述传递给求解器，进行求解

  if (status == Ipopt::Solve_Succeeded ||
      status == Ipopt::Solved_To_Acceptable_Level) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    std::cout << "*** The problem solved in " << iter_count << " iterations!";

    Ipopt::Number final_obj = app->Statistics()->FinalObjective();
    std::cout << "*** The final value of the objective function is "
              << final_obj << '.';

    //   PERF_BLOCK_END("DistanceApproachProblemSolving");
  } else {
    /*
      return detailed failure information,
      reference resource: Ipopt::ApplicationReturnStatus, https://
      www.coin-or.org/Doxygen/CoinAll/_ip_return_codes__inc_8h-source.html
    */
    std::unordered_map<int, std::string> failure_status = {
        {0, "Solve_Succeeded"},
        {1, "Solved_To_Acceptable_Level"},
        {2, "Infeasible_Problem_Detected"},
        {3, "Search_Direction_Becomes_Too_Small"},
        {4, "Diverging_Iterates"},
        {5, "User_Requested_Stop"},
        {6, "Feasible_Point_Found"},
        {-1, "Maximum_Iterations_Exceeded"},
        {-2, "Restoration_Failed"},
        {-3, "Error_In_Step_Computation"},
        {-10, "Not_Enough_Degrees_Of_Freedom"},
        {-11, "Invalid_Problem_Definition"},
        {-12, "Invalid_Option"},
        {-13, "Invalid_Number_Detected"},
        {-100, "Unrecoverable_Exception"},
        {-101, "NonIpopt_Exception_Thrown"},
        {-102, "Insufficient_Memory"},
        {-199, "Internal_Error"}};

    if (!failure_status.count(static_cast<size_t>(status))) {
      std::cout << "Solver ends with unknown failure code: "
                << static_cast<int>(status);
    } else {
      std::cout << "Solver failure case: "
                << failure_status[static_cast<size_t>(status)];
    }
  }
  Eigen::MatrixXd dual_l_result; //返回的优化结果
  Eigen::MatrixXd dual_n_result;
  ptop->get_optimization_results(state_result, control_result, time_result,
                                 &dual_l_result, &dual_n_result); //获取结果

  return status == Ipopt::Solve_Succeeded ||
         status == Ipopt::Solved_To_Acceptable_Level;
}
