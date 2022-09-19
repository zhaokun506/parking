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

#pragma once

#include <algorithm>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "configs/vehicle_config.h"
#include "configs/vehicle_config_helper.h"
#include "common/math/math_utils.h"
//#include "common/obstacle.h"
#include "coarse_trajectory_generator/grid_search.h"
#include "coarse_trajectory_generator/node3d.h"
#include "coarse_trajectory_generator/reeds_shepp_path.h"
#include "configs/planner_open_space_config.h"

struct HybridAStartResult
{
  std::vector<double> x;             // x坐标
  std::vector<double> y;             // y坐标
  std::vector<double> phi;           //横摆角
  std::vector<double> v;             //速度
  std::vector<double> a;             //加速度
  std::vector<double> steer;         //方向盘转角
  std::vector<double> accumulated_s; //累计里程
};

class HybridAStar
{
public:
  explicit HybridAStar(const PlannerOpenSpaceConfig &open_space_conf);
  virtual ~HybridAStar() = default;
  bool Plan(double sx, double sy, double sphi, double ex, double ey, //起点终点状态
            double ephi, const std::vector<double> &XYbounds,        //地图范围
            const std::vector<std::vector<common::math::Vec2d>> &
                obstacles_vertices_vec,  //障碍物顶点数组，顶点的表示法为Vec2d向量
            HybridAStartResult *result); //结果
  bool TrajectoryPartition(const HybridAStartResult &result,
                           std::vector<HybridAStartResult> *partitioned_result);

private:
  bool AnalyticExpansion(std::shared_ptr<Node3d> current_node);
  // check collision and validity
  bool ValidityCheck(std::shared_ptr<Node3d> node);
  // check Reeds Shepp path collision and validity
  bool RSPCheck(const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end);
  // load the whole RSP as nodes and add to the close set
  std::shared_ptr<Node3d> LoadRSPinCS(
      const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
      std::shared_ptr<Node3d> current_node);
  std::shared_ptr<Node3d> Next_node_generator(
      std::shared_ptr<Node3d> current_node, size_t next_node_index);
  void CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                         std::shared_ptr<Node3d> next_node);
  double TrajCost(std::shared_ptr<Node3d> current_node,
                  std::shared_ptr<Node3d> next_node);
  double HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node);
  bool GetResult(HybridAStartResult *result);
  bool GetTemporalProfile(HybridAStartResult *result);
  bool GenerateSpeedAcceleration(HybridAStartResult *result);
  bool GenerateSCurveSpeedAcceleration(HybridAStartResult *result);

private:
  PlannerOpenSpaceConfig planner_open_space_config_;
  common::VehicleParam vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  size_t next_node_num_ = 0;
  double max_steer_angle_ = 0.0;
  double step_size_ = 0.0;
  double xy_grid_resolution_ = 0.0;
  double delta_t_ = 0.0;
  double traj_forward_penalty_ = 0.0;
  double traj_back_penalty_ = 0.0;
  double traj_gear_switch_penalty_ = 0.0;
  double traj_steer_penalty_ = 0.0;
  double traj_steer_change_penalty_ = 0.0;
  double heu_rs_forward_penalty_ = 0.0;
  double heu_rs_back_penalty_ = 0.0;
  double heu_rs_gear_switch_penalty_ = 0.0;
  double heu_rs_steer_penalty_ = 0.0;
  double heu_rs_steer_change_penalty_ = 0.0;
  std::vector<double> XYbounds_;
  std::shared_ptr<Node3d> start_node_;
  std::shared_ptr<Node3d> end_node_;
  std::shared_ptr<Node3d> final_node_;
  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec_;

  struct cmp
  {
    bool operator()(const std::pair<std::string, double> &left,
                    const std::pair<std::string, double> &right) const
    {
      return left.second >= right.second;
    }
  };
  /*
  std::priority_queue:在优先队列中，优先级高的元素先出队列，并非按照先进先出的要求，类似一个堆(heap)。
  其模板声明带有三个参数，priority_queue<Type, Container, Functional>, 其中Type为数据类型，Container为保存数据的容器，Functional为元素比较方式。
  Container必须是用数组实现的容器，比如 vector, deque. STL里面默认用的是vector. 比较方式默认用operator< , 所以如果把后面两个参数缺省的话，优先队列就是大顶堆，队头元素最大。
  priority_queue()，默认按照从小到大排列。所以top()返回的是最大值而不是最小值！
  使用greater<>后，数据从大到小排列，top()返回的就是最小值而不是最大值！
  如果使用了第三个参数，那第二个参数不能省，用作保存数据的容器！！！！*/
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp> //优先队列
      open_pq_;
  std::unordered_map<std::string, std::shared_ptr<Node3d>> open_set_; //数据结构无序map
  std::unordered_map<std::string, std::shared_ptr<Node3d>> close_set_;
  std::unique_ptr<ReedShepp> reed_shepp_generator_;
  std::unique_ptr<GridSearch> grid_a_star_heuristic_generator_;
};
