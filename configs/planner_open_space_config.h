#pragma once

#include <string>
//速度优化参数配置
struct sCurveConfig {
  double acc_weight = 1.0;             //加速度权重
  double jerk_weight = 0.0;            //加加速度权重
  double kappa_penalty_weight = 100.0; //曲率权重
  double ref_s_weight = 0.1;           //参数s权重
  double ref_v_weight = 0.0;           //参考速度权重
};
// A*算法参数配置
struct HybridAstar {
  double xy_grid_resolution = 0.4;  // xy栅格分辨率 m
  double phi_grid_resolution = 0.1; //φ栅格分辨率 rad
  double next_node_num = 10;        //扩展的节点数量 上5下5
  double step_size = 0.2;           //步长

  double traj_forward_penalty = 1.0; //
  double traj_back_penalty = 1.0;
  double traj_gear_switch_penalty = 10.0; //行驶方向切换惩罚
  double traj_steer_penalty = 0;          //方向盘转角惩罚
  double traj_steer_change_penalty = 0.1; //方向盘方向切换惩罚

  double grid_a_star_xy_resolution = 0.4; // A*xy分辨率
  double node_radius = 0.25;              //
  double delta_t = 1;
  sCurveConfig s_curve_config;
};

struct IpoptConfig {
  double ipopt_print_level = 0;
  double mumps_mem_percent = 6000;
  double mumps_pivtol = 1e-06;
  double ipopt_max_iter = 2000;
  double ipopt_tol = 0.0001;
  double ipopt_acceptable_constr_viol_tol = 0.1;
  double ipopt_min_hessian_perturbation = 1e-12;
  double ipopt_jacobian_regularization_value = 1e-07;
  std::string ipopt_print_timing_statistics = "yes";
  std::string ipopt_alpha_for_y = "min";
  std::string ipopt_recalc_y = "yes";
  double ipopt_mu_init = 0.1;
};

struct DistanceApproachConfig {
  double weight_steer = 0.3;
  double weight_a = 1.0;        // 1.271
  double weight_steer_rate = 2; // 2
  double weight_a_rate = 2.5;   // 2.5
  double weight_x = 9;          // 18        //符合A*路径代价
  double weight_y = 7;          // 14
  double weight_phi = 5;        // 10
  double weight_v = 0.0;
  double weight_steer_stitching = 1.75; // defualt 1.75
  double weight_a_stitching = 3.25;     // 3.25
  double weight_first_order_time = 1;   //时间
  double weight_second_order_time = 2.0;
  double weight_end_state = 1.0;
  double weight_slack = 1.0;
  double min_safety_distance = 0.01;
  double max_speed_forward = 2.0;
  double max_speed_reverse = 1.0;
  double max_acceleration_forward = 2.0;
  double max_acceleration_reverse = 1.0;
  double min_time_sample_scaling = 0.5;
  double max_time_sample_scaling = 1.5;
  double use_fix_time = false;

  IpoptConfig ipopt_config;

  bool enable_constraint_check = false;
  bool enable_hand_derivative = false;
  bool enable_derivative_check = false;
  bool enable_initial_final_check = false;
  //终点放松
  // std::string distance_approach_mode = "DISTANCE_APPROACH_IPOPT_RELAX_END";
  //可变ts优化，求解失败
  // std::string distance_approach_mode = "DISTANCE_APPROACH_IPOPT";
  //安全距离可调
  std::string distance_approach_mode =
      "DISTANCE_APPROACH_IPOPT_RELAX_END_SLACK";
  bool enable_check_initial_state = false;
  bool enable_jacobian_ad = true;
};

struct OsqpConfig {
  double alpha = 1.0;
  double eps_abs = 1.0e-3;
  double eps_rel = 1.0e-3;
  double max_iter = 10000;
  bool polish = true;
  bool osqp_debug_log = false;
};

struct DualVariableWarmStartConfig {

  double weight_d = 1.0;
  std::string qp_format = "OSQP";
  OsqpConfig osqp_config;
  double min_safety_distance = 0.1; // deflaut=0.01
  bool debug_osqp = false;
  double beta = 1.0;
};

struct IterativeAnchoringSmootherConfig {
  double interpolated_delta_s = 0.1;
  double reanchoring_trails_num = 50;
  double reanchoring_pos_stddev = 0.25;
  double reanchoring_length_stddev = 1.0;
  bool estimate_bound = false;
  double default_bound = 2.0;
  double vehicle_shortest_dimension = 1.04;

  double collision_decrease_ratio = 0.9;
  double max_forward_v = 2.0;
  double max_reverse_v = 2.0;
  double max_forward_acc = 3.0;
  double max_reverse_acc = 2.0;
  double max_acc_jerk = 4.0;
  double delta_t = 0.2;
  sCurveConfig s_curve_config;
};

class PlannerOpenSpaceConfig {
private:
  /* data */
public:
  PlannerOpenSpaceConfig() = default;
  ~PlannerOpenSpaceConfig() = default;

  HybridAstar warm_start_config;
  DistanceApproachConfig distance_approach_config;
  DualVariableWarmStartConfig dual_variable_warm_start_config;
  IterativeAnchoringSmootherConfig iterative_anchoring_smoother_config;
  double is_near_destination_threshold = 0.1;
  double delta_t = 0.1;
};

// //类的成员变量是否能够在定义时设置初值

// PlannerOpenSpaceConfig::PlannerOpenSpaceConfig() {}
