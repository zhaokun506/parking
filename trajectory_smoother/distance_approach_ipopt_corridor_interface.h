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
 distance_approach方法的ipopt优化接口
 */

#pragma once
#include <iostream>
#include <omp.h>

#include <algorithm>
#include <limits>
#include <vector>

#include <adolc/adolc.h>
#include <adolc/adolc_openmp.h>
#include <adolc/adolc_sparse.h>
#include <adolc/adouble.h>
#include <coin-or/IpTNLP.hpp>
#include <coin-or/IpTypes.hpp>

#include "eigen3/Eigen/Dense"

#include "configs/planner_open_space_config.h"
#include "configs/vehicle_config.h"

#include "common/math/math_utils.h"
#include "configs/vehicle_config_helper.h"
//#include "modules/common/util/util.h"
//#include "modules/planning/common/planning_gflags.h"
#include "trajectory_smoother/distance_approach_interface.h"

#define tag_f 1
#define tag_g 2
#define tag_L 3
#define HPOFF 30

class DistanceApproachIPOPTCorridorInterface
    : public DistanceApproachInterface {
public:
  DistanceApproachIPOPTCorridorInterface(
      const size_t horizon, const double ds_init, const Eigen::MatrixXd &ego,
      const Eigen::MatrixXd &xWS, const Eigen::MatrixXd &uWS,
      const Eigen::MatrixXd &x0, const Eigen::MatrixXd &xf,
      const Eigen::MatrixXd &last_time_u, const std::vector<double> &XYbounds,
      const Eigen::MatrixXi &obstacles_edges_num, const size_t obstacles_num,
      const Eigen::MatrixXd &f_driving_bound,
      const Eigen::MatrixXd &b_driving_bound,
      const PlannerOpenSpaceConfig &planner_open_space_config);

  ~DistanceApproachIPOPTCorridorInterface() = default;

  /** Method to return some info about the nlp */ //获取npl信息
  bool get_nlp_info(int &n, int &m, int &nnz_jac_g, int &nnz_h_lag, // NOLINT
                    IndexStyleEnum &index_style) override;          // NOLINT

  /** Method to return the bounds for my problem */ //获得边界信息
  bool get_bounds_info(int n, double *x_l, double *x_u, int m, double *g_l,
                       double *g_u) override;

  /** Method to return the starting point for the algorithm */ //获取初始解
  bool get_starting_point(int n, bool init_x, double *x, bool init_z,
                          double *z_L, double *z_U, int m, bool init_lambda,
                          double *lambda) override;

  /** Method to return the objective value */ //代价函数
  bool eval_f(int n, const double *x, bool new_x, double &obj_value) override;

  /** Method to return the gradient of the objective */ //代价函数梯度（目标方程偏导方程)
  bool eval_grad_f(int n, const double *x, bool new_x, double *grad_f) override;

  /** Method to return the constraint residuals */ //定义约束函数
  bool eval_g(int n, const double *x, bool new_x, int m, double *g) override;

  /** Check unfeasible constraints for further study**/
  bool check_g(int n, const double *x, int m, const double *g);

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is nullptr)
   *   2) The values of the jacobian (if "values" is not nullptr)
   */
  //定义约束函数的雅克比矩阵
  bool eval_jac_g(int n, const double *x, bool new_x, int m, int nele_jac,
                  int *iRow, int *jCol, double *values) override;

  // sequential implementation to jac_g
  bool eval_jac_g_ser(int n, const double *x, bool new_x, int m, int nele_jac,
                      int *iRow, int *jCol, double *values) override;

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is
   * nullptr) 2) The values of the hessian of the lagrangian (if "values" is not
   * nullptr)
   */
  //海森矩阵
  bool eval_h(int n, const double *x, bool new_x, double obj_factor, int m,
              const double *lambda, bool new_lambda, int nele_hess, int *iRow,
              int *jCol, double *values) override;

  /** @name Solution Methods */ //求解NLP问题并输出最优解向量
  /** This method is called when the algorithm is complete so the TNLP can
   * store/write the solution */
  void finalize_solution(Ipopt::SolverReturn status, int n, const double *x,
                         const double *z_L, const double *z_U, int m,
                         const double *g, const double *lambda,
                         double obj_value, const Ipopt::IpoptData *ip_data,
                         Ipopt::IpoptCalculatedQuantities *ip_cq) override;
  //获取求解结果
  void get_optimization_results(Eigen::MatrixXd *state_result,
                                Eigen::MatrixXd *control_result,
                                Eigen::MatrixXd *time_result,
                                Eigen::MatrixXd *dual_l_result,
                                Eigen::MatrixXd *dual_n_result) const override;

  //***************    start ADOL-C part ***********************************
  //自动微分工具
  /** Template to return the objective value */ //将代价函数改写为模板函数
  template <class T> void eval_obj(int n, const T *x, T *obj_value);

  /** Template to compute constraints */ //将约束函数改写为模板函数
  template <class T> void eval_constraints(int n, const T *x, int m, T *g);

  /** Method to generate the required tapes by ADOL-C*/ //自动微分记录函数，由get_nlp_info调用，记录相关信息
  void generate_tapes(int n, int m, int *nnz_jac_g, int *nnz_h_lag);
  //***************    end   ADOL-C part ***********************************

private:
  int num_of_variables_ = 0;
  int num_of_constraints_ = 0;
  int horizon_ = 0;
  int lambda_horizon_ = 0;
  int miu_horizon_ = 0;
  double ds_init_ = 0.1;
  Eigen::MatrixXd ego_;
  Eigen::MatrixXd xWS_;
  Eigen::MatrixXd uWS_;
  Eigen::MatrixXd l_warm_up_;
  Eigen::MatrixXd n_warm_up_;
  Eigen::MatrixXd x0_;
  Eigen::MatrixXd xf_;
  Eigen::MatrixXd last_time_u_;
  std::vector<double> XYbounds_;

  double ts_;

  // debug flag
  bool enable_constraint_check_;

  // penalty
  double weight_state_x_ = 0.0;
  double weight_state_y_ = 0.0;
  double weight_state_phi_ = 0.0;
  double weight_state_v_ = 0.0;
  double weight_input_steer_ = 0.0;
  double weight_input_a_ = 0.0;
  double weight_rate_steer_ = 0.0;
  double weight_rate_a_ = 0.0;
  double weight_stitching_steer_ = 0.0;
  double weight_stitching_a_ = 0.0;
  double weight_first_order_time_ = 0.0;
  double weight_second_order_time_ = 0.0;

  double w_ev_ = 0.0;
  double l_ev_ = 0.0;
  std::vector<double> g_;
  double offset_ = 0.0;
  Eigen::MatrixXi obstacles_edges_num_;
  int obstacles_num_ = 0;
  int obstacles_edges_sum_ = 0;
  double wheelbase_ = 0.0;

  Eigen::MatrixXd state_result_;
  // Eigen::MatrixXd dual_l_result_;
  // Eigen::MatrixXd dual_n_result_;
  Eigen::MatrixXd control_result_;
  Eigen::MatrixXd time_result_;

  // obstacles_A
  Eigen::MatrixXd obstacles_A_;

  // obstacles_b
  Eigen::MatrixXd obstacles_b_;

  // obstacles_A
  Eigen::MatrixXd f_driving_bound_;

  // obstacles_b
  Eigen::MatrixXd b_driving_bound_;

  // whether to use fix time
  bool use_fix_time_ = false;

  // state start index
  int state_start_index_ = 0;

  // control start index.
  int control_start_index_ = 0;

  // time start index
  int time_start_index_ = 0;

  // lagrangian l start index
  int l_start_index_ = 0;

  // lagrangian n start index
  int n_start_index_ = 0;

  double min_safety_distance_ = 0.0;

  double max_safety_distance_ = 0.0;

  double max_steer_angle_ = 0.0;

  double max_speed_forward_ = 0.0;

  double max_speed_reverse_ = 0.0;

  double max_acceleration_forward_ = 0.0;

  double max_acceleration_reverse_ = 0.0;

  double min_time_sample_scaling_ = 0.0;

  double max_time_sample_scaling_ = 0.0;

  double max_steer_rate_ = 0.0;

  double max_lambda_ = 0.0;

  double max_miu_ = 0.0;

  bool enable_jacobian_ad_ = true;

private:
  DistanceApproachConfig distance_approach_config_;
  const VehicleParam vehicle_param_;
  ;

private:
  //***************    start ADOL-C part ***********************************
  double *obj_lam;
  //** variables for sparsity exploitation
  unsigned int *rind_g; /* row indices 雅克比矩阵行记录    */
  unsigned int *cind_g; /* column indices 雅克比矩阵列记录*/
  double *jacval;       /* values  雅克比矩阵的列     */
  unsigned int *rind_L; /* row indices  拉格朗日矩阵行  */
  unsigned int *cind_L; /* column indices 拉格朗日列 */
  double *hessval;      /* values  拉格朗日值*/
  int nnz_jac;          //约束的雅克比矩阵非零数
  int nnz_L;            //海森矩阵非零数
  int options_g[4];
  int options_L[4];
  //***************    end   ADOL-C part ***********************************
};
