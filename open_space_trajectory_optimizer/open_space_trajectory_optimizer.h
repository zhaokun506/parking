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

#pragma once

#include <memory>
#include <vector>

#include "eigen3/Eigen/Eigen"

#ifdef ALIVE
#undef ALIVE
#endif

#include "configs/vehicle_config.h"
//#include "modules/common/vehicle_state/proto/vehicle_state.h"
#include "coarse_trajectory_generator/hybrid_a_star.h"
#include "common/math/vec2d.h"
#include "common/pnc_point.h"
//#include "configs/open_space_task_config.h"
#include "configs/open_space_trajectory_optimizer_config.h"
//#include "node3d.h"
#include "open_space_map/open_space_map.h"
#include "planning_data/trajectory/discretized_trajectory.h"
#include "trajectory_smoother/construct_driving_corridor.h"
#include "trajectory_smoother/distance_approach_problem.h"

//#include "trajectory_smoother/iterative_anchoring_smoother.h"

class OpenSpaceTrajectoryOptimizer {
public:
  OpenSpaceTrajectoryOptimizer(
      const OpenSpaceTrajectoryOptimizerConfig &config);

  virtual ~OpenSpaceTrajectoryOptimizer() = default;
  //类的主函数
  int Plan(
        const MapPoint &start_pose, //起点
        const MapPoint &end_pose,
        const std::vector<double> &XYbounds, // 4,XYbounds in xmin, xmax, ymin, ymax
        double rotate_angle, const common::math::Vec2d &translate_origin,
        const Eigen::MatrixXi &obstacles_edges_num,                                         // H表示中A和b矩阵维数所需的维数
        /*const Eigen::MatrixXd &f_driving_bound, const Eigen::MatrixXd &b_driving_bound,*/ //障碍物Ax＞b的线性不等式表示，此处在混合A*之后生产行车隧道前后行车隧道链两个约束矩阵
        const std::vector<std::vector<common::math::Vec2d>> &obstacles_vertices_vec,        //以逆时钟顺序存储障碍物顶点的向量
        double *time_latency);
  void GetOptimizedTrajectory(DiscretizedTrajectory *optimized_trajectory);
  void GetCoarseTrajectory(DiscretizedTrajectory *optimized_trajectory);
  Eigen::MatrixXd GetFrontDrivingBound();
  Eigen::MatrixXd GetBackDrivingBound();
  std::shared_ptr<ConstructDrivingCorridor> GetConstructCorridorPtr();

private:
  void LoadTrajectory(const Eigen::MatrixXd &state_result_ds,
                      const Eigen::MatrixXd &control_result_ds,
                      const Eigen::MatrixXd &time_result_ds);

  void LoadCoarseTrajectory(const Eigen::MatrixXd &state_result_ds,
                            const Eigen::MatrixXd &control_result_ds,
                            const Eigen::MatrixXd &time_result_ds);

  void LoadHybridAstarResultInEigen(HybridAStartResult *result,
                                    Eigen::MatrixXd *xWS, Eigen::MatrixXd *uWS);

  bool GenerateDistanceApproachTraj(
      const Eigen::MatrixXd &xWS, const Eigen::MatrixXd &uWS,
      const std::vector<double> &XYbounds,
      const Eigen::MatrixXi &obstacles_edges_num,
      const Eigen::MatrixXd &f_driving_bound,
      const Eigen::MatrixXd &b_driving_bound,
      const std::vector<std::vector<common::math::Vec2d>>
          &obstacles_vertices_vec,
      const Eigen::MatrixXd &last_time_u, const double init_v,
      Eigen::MatrixXd *state_result_ds, Eigen::MatrixXd *control_result_ds,
      Eigen::MatrixXd *time_result_ds);

  bool GenerateCoarseTraj(const Eigen::MatrixXd &xWS,
                          const Eigen::MatrixXd &uWS,
                          const std::vector<double> &XYbounds,
                          const Eigen::MatrixXi &obstacles_edges_num,
                          const Eigen::MatrixXd &f_driving_bound,
                          const Eigen::MatrixXd &b_driving_bound,
                          const std::vector<std::vector<common::math::Vec2d>>
                              &obstacles_vertices_vec,
                          const Eigen::MatrixXd &last_time_u,
                          const double init_v, Eigen::MatrixXd *state_result_ds,
                          Eigen::MatrixXd *control_result_ds,
                          Eigen::MatrixXd *time_result_ds);

  bool GenerateDecoupledTraj(const Eigen::MatrixXd &xWS, const double init_a,
                             const double init_v,
                             const std::vector<std::vector<common::math::Vec2d>>
                                 &obstacles_vertices_vec,
                             Eigen::MatrixXd *state_result_dc,
                             Eigen::MatrixXd *control_result_dc,
                             Eigen::MatrixXd *time_result_dc);

  void LoadResult(const DiscretizedTrajectory &discretized_trajectory,
                  Eigen::MatrixXd *state_result_dc,
                  Eigen::MatrixXd *control_result_dc,
                  Eigen::MatrixXd *time_result_dc);
  void UseWarmStartAsResult(const Eigen::MatrixXd &xWS,
                            const Eigen::MatrixXd &uWS,
                            Eigen::MatrixXd *state_result_ds,
                            Eigen::MatrixXd *control_result_ds,
                            Eigen::MatrixXd *time_result_ds);

private:
  OpenSpaceTrajectoryOptimizerConfig config_;
  VehicleParam vehicle_param_;
  //对于其他类的调用，采用将其他类定义为私有成员变量的方式
  std::unique_ptr<HybridAStar> warm_start_;                      //混合A*，
  std::shared_ptr<ConstructDrivingCorridor> construct_corridor_; //构建行车隧道
  std::unique_ptr<DistanceApproachProblem> distance_approach_; //距离接近问题

  //输出的结果，定义为私有变量，使用同名的Get方法读取
  std::vector<common::TrajectoryPoint> stitching_trajectory_;
  DiscretizedTrajectory optimized_trajectory_;
  DiscretizedTrajectory coarse_trajectory_;

  // 1.车辆前圆心可行驶边界n*4矩阵,n*(x_min,x_max,y_min,y_max)
  Eigen::MatrixXd f_bound_;
  Eigen::MatrixXd r_bound_;
};
