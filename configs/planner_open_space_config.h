#pragma once

#include <string>
//速度优化参数配置
struct sCurveConfig {
  const double acc_weight = 1.0;             //加速度权重
  const double jerk_weight = 0.0;            //加加速度权重
  const double kappa_penalty_weight = 100.0; //曲率权重
  const double ref_s_weight = 0.1;           //参数s权重
  const double ref_v_weight = 0.0;           //参考速度权重
};
// A*算法参数配置
struct HybridAstar {
  const double xy_grid_resolution = 0.3;  // xy栅格分辨率 m
  const double phi_grid_resolution = 0.1;  //φ栅格分辨率 rad
  const double next_node_num = 10;         //扩展的节点数量 上5下5
  const double step_size = 0.5;            //步长
  const double traj_forward_penalty = 1.0; //
  const double traj_back_penalty = 1.0;
  const double traj_gear_switch_penalty = 10.0; //行驶方向切换惩罚
  const double traj_steer_penalty = 0.0;        //方向盘转角惩罚
  const double traj_steer_change_penalty = 0.0; //方向盘方向切换惩罚
  const double grid_a_star_xy_resolution = 0.5; // A*xy分辨率
  const double node_radius = 0.25;              //
  const double delta_t = 0.1;
  sCurveConfig s_curve_config;
};

struct IpoptConfig {
  const double ipopt_print_level = 0;
  const double mumps_mem_percent = 6000;
  const double mumps_pivtol = 1e-06;
  const double ipopt_max_iter = 1000;
  const double ipopt_tol = 0.0001;
  const double ipopt_acceptable_constr_viol_tol = 0.1;
  const double ipopt_min_hessian_perturbation = 1e-12;
  const double ipopt_jacobian_regularization_value = 1e-07;
  std::string ipopt_print_timing_statistics = "yes";
  std::string ipopt_alpha_for_y = "min";
  std::string ipopt_recalc_y = "yes";
  double ipopt_mu_init = 0.1;
};

struct DistanceApproachConfig {
  const double weight_steer = 0.3;
  const double weight_a = 1.1;
  const double weight_steer_rate = 2.0;
  const double weight_a_rate = 2.5;
  const double weight_x = 18.0;
  const double weight_y = 14.0;
  const double weight_phi = 10.0;
  const double weight_v = 0.0;
  const double weight_steer_stitching = 1.75;
  const double weight_a_stitching = 3.25;
  const double weight_first_order_time = 1.0;
  const double weight_second_order_time = 2.0;
  const double weight_end_state = 1.0;
  const double weight_slack = 1.0;
  const double min_safety_distance = 0.01;
  const double max_speed_forward = 2.0;
  const double max_speed_reverse = 1.0;
  const double max_acceleration_forward = 2.0;
  const double max_acceleration_reverse = 1.0;
  const double min_time_sample_scaling = 0.5;
  const double max_time_sample_scaling = 1.5;
  const double use_fix_time = false;

  IpoptConfig ipopt_config;

  bool enable_constraint_check = false;
  bool enable_hand_derivative = false;
  bool enable_derivative_check = false;
  bool enable_initial_final_check = false;
  std::string distance_approach_mode = "DISTANCE_APPROACH_CORRIDOR_IPOPT";
  bool enable_check_initial_state = false;
  bool enable_jacobian_ad = true;
};

class PlannerOpenSpaceConfig {
private:
  /* data */
public:
  PlannerOpenSpaceConfig() = default;
  ~PlannerOpenSpaceConfig() = default;

  HybridAstar warm_start_config;
  DistanceApproachConfig distance_approach_config;
};

// //类的成员变量是否能够在定义时设置初值

// PlannerOpenSpaceConfig::PlannerOpenSpaceConfig() {}
