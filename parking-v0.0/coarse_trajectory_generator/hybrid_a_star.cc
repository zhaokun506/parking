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

#include "coarse_trajectory_generator/hybrid_a_star.h"
#include "common/piecewise_jerk/piecewise_jerk_speed_problem.h"

namespace apollo
{
  namespace planning
  {

    using apollo::common::math::Box2d;
    using apollo::common::math::Vec2d;
    using apollo::cyber::Clock;
    //在构造函数中初始化，初始化获取一些配置
    HybridAStar::HybridAStar(const PlannerOpenSpaceConfig &open_space_conf)
    {
      planner_open_space_config_.CopyFrom(open_space_conf);
      reed_shepp_generator_ =
          std::make_unique<ReedShepp>(vehicle_param_, planner_open_space_config_);
      grid_a_star_heuristic_generator_ =
          std::make_unique<GridSearch>(planner_open_space_config_);
      next_node_num_ = //扩展节点的数量
          planner_open_space_config_.warm_start_config().next_node_num();
      max_steer_angle_ =
          vehicle_param_.max_steer_angle() / vehicle_param_.steer_ratio();
      step_size_ = planner_open_space_config_.warm_start_config().step_size(); //行驶步长
      xy_grid_resolution_ =
          planner_open_space_config_.warm_start_config().xy_grid_resolution();
      delta_t_ = planner_open_space_config_.delta_t();
      traj_forward_penalty_ =
          planner_open_space_config_.warm_start_config().traj_forward_penalty();
      traj_back_penalty_ =
          planner_open_space_config_.warm_start_config().traj_back_penalty();
      traj_gear_switch_penalty_ =
          planner_open_space_config_.warm_start_config().traj_gear_switch_penalty();
      traj_steer_penalty_ =
          planner_open_space_config_.warm_start_config().traj_steer_penalty();
      traj_steer_change_penalty_ = planner_open_space_config_.warm_start_config()
                                       .traj_steer_change_penalty();
    }

    bool HybridAStar::AnalyticExpansion(std::shared_ptr<Node3d> current_node)
    {
      std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
          std::make_shared<ReedSheppPath>();
      if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_,
                                              reeds_shepp_to_check))
      { //搜寻最短的RS路径
        ADEBUG << "ShortestRSP failed";
        return false;
      }

      if (!RSPCheck(reeds_shepp_to_check))
      {
        return false;
      }

      ADEBUG << "Reach the end configuration with Reed Sharp";
      // load the whole RSP as nodes and add to the close set//载入RSP作为节点并加入close集合
      final_node_ = LoadRSPinCS(reeds_shepp_to_check, current_node);
      return true;
    }

    bool HybridAStar::RSPCheck(
        const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end)
    {
      std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
          reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
          XYbounds_, planner_open_space_config_));
      return ValidityCheck(node);
    }

    bool HybridAStar::ValidityCheck(std::shared_ptr<Node3d> node)
    {
      CHECK_NOTNULL(node);
      CHECK_GT(node->GetStepSize(), 0U);

      if (obstacles_linesegments_vec_.empty())
      {
        return true; //没有障碍物直接返回true
      }

      size_t node_step_size = node->GetStepSize();
      const auto &traversed_x = node->GetXs();
      const auto &traversed_y = node->GetYs();
      const auto &traversed_phi = node->GetPhis();

      // The first {x, y, phi} is collision free unless they are start and end
      // configuration of search problem
      size_t check_start_index = 0;
      if (node_step_size == 1)
      {
        check_start_index = 0;
      }
      else
      {
        check_start_index = 1;
      }

      for (size_t i = check_start_index; i < node_step_size; ++i)
      {
        if (traversed_x[i] > XYbounds_[1] || traversed_x[i] < XYbounds_[0] ||
            traversed_y[i] > XYbounds_[3] || traversed_y[i] < XYbounds_[2])
        {
          return false;
        }
        Box2d bounding_box = Node3d::GetBoundingBox(
            vehicle_param_, traversed_x[i], traversed_y[i], traversed_phi[i]);
        //是否可以取就近的障碍物线段做碰撞检测，这样遍历所有障碍物计算量是否太大
        for (const auto &obstacle_linesegments : obstacles_linesegments_vec_)
        {
          for (const common::math::LineSegment2d &linesegment :
               obstacle_linesegments)
          {
            if (bounding_box.HasOverlap(linesegment))
            { //碰撞
              ADEBUG << "collision start at x: " << linesegment.start().x();
              ADEBUG << "collision start at y: " << linesegment.start().y();
              ADEBUG << "collision end at x: " << linesegment.end().x();
              ADEBUG << "collision end at y: " << linesegment.end().y();
              return false;
            }
          }
        }
      }
      return true;
    }

    std::shared_ptr<Node3d> HybridAStar::LoadRSPinCS(
        const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
        std::shared_ptr<Node3d> current_node)
    {
      std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(new Node3d(
          reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
          XYbounds_, planner_open_space_config_));
      end_node->SetPre(current_node); // end_node的父节点设置为当前节点
      close_set_.emplace(end_node->GetIndex(), end_node);
      return end_node;
    }
    // 3D，邻域点生成
    std::shared_ptr<Node3d> HybridAStar::Next_node_generator(
        std::shared_ptr<Node3d> current_node, size_t next_node_index)
    {
      double steering = 0.0;
      double traveled_distance = 0.0;
      // steering angle计算原理
      //首先，根据next_node_index与next_node_num_的对比是可以区分运动方向的
      //这里的if-else就是区分运动正反方向讨论的（前进和倒车）
      //其次，车辆在当前的姿态下，既可以向左转、又可以向右转，那么steering angle的
      //取值范围其实是[-max_steer_angle_, max_steer_angle_]，在正向或反向下，
      //能取next_node_num_/2个有效值。
      //即，把[-max_steer_angle_, max_steer_angle_]分为（next_node_num_/2-1）份
      //所以，steering = 初始偏移量 + 单位间隔 × index

      if (next_node_index < static_cast<double>(next_node_num_) / 2)
      { //正向
        steering =
            -max_steer_angle_ +
            (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
                static_cast<double>(next_node_index);
        traveled_distance = step_size_;
      }
      else
      { //反向
        size_t index = next_node_index - next_node_num_ / 2;
        steering =
            -max_steer_angle_ +
            (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
                static_cast<double>(index);
        traveled_distance = -step_size_;
      }
      // take above motion primitive to generate a curve driving the car to a
      // different grid按照上面的运动方向，将车辆行驶向不同的栅格
      double arc = std::sqrt(2) * xy_grid_resolution_; // arc是根号2倍的栅格分辨率
      std::vector<double> intermediate_x;
      std::vector<double> intermediate_y;
      std::vector<double> intermediate_phi;
      double last_x = current_node->GetX();
      double last_y = current_node->GetY();
      double last_phi = current_node->GetPhi();
      intermediate_x.push_back(last_x);
      intermediate_y.push_back(last_y);
      intermediate_phi.push_back(last_phi);
      for (size_t i = 0; i < arc / step_size_; ++i)
      { //寻找行走超过>栅格根号2倍的距离的作为next_node
        const double next_x = last_x + traveled_distance * std::cos(last_phi);
        const double next_y = last_y + traveled_distance * std::sin(last_phi);
        const double next_phi = common::math::NormalizeAngle(
            last_phi +
            traveled_distance / vehicle_param_.wheel_base() * std::tan(steering));
        intermediate_x.push_back(next_x);
        intermediate_y.push_back(next_y);
        intermediate_phi.push_back(next_phi);
        last_x = next_x;
        last_y = next_y;
        last_phi = next_phi;
      }
      // check if the vehicle runs outside of XY boundary检测是否超出边界
      if (intermediate_x.back() > XYbounds_[1] ||
          intermediate_x.back() < XYbounds_[0] ||
          intermediate_y.back() > XYbounds_[3] ||
          intermediate_y.back() < XYbounds_[2])
      {
        return nullptr;
      }
      std::shared_ptr<Node3d> next_node = std::shared_ptr<Node3d>(
          new Node3d(intermediate_x, intermediate_y, intermediate_phi, XYbounds_,
                     planner_open_space_config_));
      next_node->SetPre(current_node);
      next_node->SetDirec(traveled_distance > 0.0);
      next_node->SetSteer(steering);
      return next_node;
    }

    void HybridAStar::CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                                        std::shared_ptr<Node3d> next_node)
    {
      next_node->SetTrajCost(current_node->GetTrajCost() +
                             TrajCost(current_node, next_node)); //上一节点的轨迹代价+两个节点之间的轨迹代价g
      // evaluate heuristic cost启发cost h
      double optimal_path_cost = 0.0;
      optimal_path_cost += HoloObstacleHeuristic(next_node);
      next_node->SetHeuCost(optimal_path_cost);
    }

    double HybridAStar::TrajCost(std::shared_ptr<Node3d> current_node,
                                 std::shared_ptr<Node3d> next_node)
    {
      // evaluate cost on the trajectory and add current cost
      double piecewise_cost = 0.0;
      if (next_node->GetDirec())
      {
        piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                          step_size_ * traj_forward_penalty_;
      }
      else
      {
        piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                          step_size_ * traj_back_penalty_;
      }
      if (current_node->GetDirec() != next_node->GetDirec())
      {
        piecewise_cost += traj_gear_switch_penalty_;
      }
      piecewise_cost += traj_steer_penalty_ * std::abs(next_node->GetSteer());
      piecewise_cost += traj_steer_change_penalty_ *
                        std::abs(next_node->GetSteer() - current_node->GetSteer());
      return piecewise_cost;
    }

    double HybridAStar::HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node)
    {
      return grid_a_star_heuristic_generator_->CheckDpMap(next_node->GetX(),
                                                          next_node->GetY());
    }

    bool HybridAStar::GetResult(HybridAStartResult *result)
    {
      std::shared_ptr<Node3d> current_node = final_node_;
      std::vector<double> hybrid_a_x;
      std::vector<double> hybrid_a_y;
      std::vector<double> hybrid_a_phi;
      while (current_node->GetPreNode() != nullptr)
      {
        std::vector<double> x = current_node->GetXs();
        std::vector<double> y = current_node->GetYs();
        std::vector<double> phi = current_node->GetPhis();
        if (x.empty() || y.empty() || phi.empty())
        {
          AERROR << "result size check failed";
          return false;
        }
        if (x.size() != y.size() || x.size() != phi.size())
        {
          AERROR << "states sizes are not equal";
          return false;
        }
        std::reverse(x.begin(), x.end());
        std::reverse(y.begin(), y.end());
        std::reverse(phi.begin(), phi.end());
        x.pop_back();
        y.pop_back();
        phi.pop_back();
        hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
        hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
        hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
        current_node = current_node->GetPreNode();
      }
      hybrid_a_x.push_back(current_node->GetX());
      hybrid_a_y.push_back(current_node->GetY());
      hybrid_a_phi.push_back(current_node->GetPhi());
      std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
      std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
      std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
      (*result).x = hybrid_a_x;
      (*result).y = hybrid_a_y;
      (*result).phi = hybrid_a_phi;

      if (!GetTemporalProfile(result))
      {
        AERROR << "GetSpeedProfile from Hybrid Astar path fails";
        return false;
      }

      if (result->x.size() != result->y.size() ||
          result->x.size() != result->v.size() ||
          result->x.size() != result->phi.size())
      {
        AERROR << "state sizes not equal, "
               << "result->x.size(): " << result->x.size() << "result->y.size()"
               << result->y.size() << "result->phi.size()" << result->phi.size()
               << "result->v.size()" << result->v.size();
        return false;
      }
      if (result->a.size() != result->steer.size() ||
          result->x.size() - result->a.size() != 1)
      {
        AERROR << "control sizes not equal or not right";
        AERROR << " acceleration size: " << result->a.size();
        AERROR << " steer size: " << result->steer.size();
        AERROR << " x size: " << result->x.size();
        return false;
      }
      return true;
    }

    bool HybridAStar::GenerateSpeedAcceleration(HybridAStartResult *result)
    {
      // Sanity Check
      if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2)
      {
        AERROR << "result size check when generating speed and acceleration fail";
        return false;
      }
      const size_t x_size = result->x.size();

      // load velocity from position
      // initial and end speed are set to be zeros
      result->v.push_back(0.0);
      for (size_t i = 1; i + 1 < x_size; ++i)
      {
        double discrete_v = (((result->x[i + 1] - result->x[i]) / delta_t_) *
                                 std::cos(result->phi[i]) +
                             ((result->x[i] - result->x[i - 1]) / delta_t_) *
                                 std::cos(result->phi[i])) /
                                2.0 +
                            (((result->y[i + 1] - result->y[i]) / delta_t_) *
                                 std::sin(result->phi[i]) +
                             ((result->y[i] - result->y[i - 1]) / delta_t_) *
                                 std::sin(result->phi[i])) /
                                2.0;
        result->v.push_back(discrete_v);
      }
      result->v.push_back(0.0);

      // load acceleration from velocity
      for (size_t i = 0; i + 1 < x_size; ++i)
      {
        const double discrete_a = (result->v[i + 1] - result->v[i]) / delta_t_;
        result->a.push_back(discrete_a);
      }

      // load steering from phi
      for (size_t i = 0; i + 1 < x_size; ++i)
      {
        double discrete_steer = (result->phi[i + 1] - result->phi[i]) *
                                vehicle_param_.wheel_base() / step_size_;
        if (result->v[i] > 0.0)
        {
          discrete_steer = std::atan(discrete_steer);
        }
        else
        {
          discrete_steer = std::atan(-discrete_steer);
        }
        result->steer.push_back(discrete_steer);
      }
      return true;
    }

    bool HybridAStar::GenerateSCurveSpeedAcceleration(HybridAStartResult *result)
    {
      // sanity check
      CHECK_NOTNULL(result);
      if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2)
      {
        AERROR << "result size check when generating speed and acceleration fail";
        return false;
      }
      if (result->x.size() != result->y.size() ||
          result->x.size() != result->phi.size())
      {
        AERROR << "result sizes not equal";
        return false;
      }

      // get gear info
      double init_heading = result->phi.front();
      const Vec2d init_tracking_vector(result->x[1] - result->x[0],
                                       result->y[1] - result->y[0]);
      const double gear =
          std::abs(common::math::NormalizeAngle(
              init_heading - init_tracking_vector.Angle())) < M_PI_2;

      // get path lengh
      size_t path_points_size = result->x.size();

      double accumulated_s = 0.0;
      result->accumulated_s.clear();
      auto last_x = result->x.front();
      auto last_y = result->y.front();
      for (size_t i = 0; i < path_points_size; ++i)
      {
        double x_diff = result->x[i] - last_x;
        double y_diff = result->y[i] - last_y;
        accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
        result->accumulated_s.push_back(accumulated_s);
        last_x = result->x[i];
        last_y = result->y[i];
      }
      // assume static initial state
      const double init_v = 0.0;
      const double init_a = 0.0;

      // minimum time speed optimization
      // TODO(Jinyun): move to confs
      const double max_forward_v = 2.0;
      const double max_reverse_v = 1.0;
      const double max_forward_acc = 2.0;
      const double max_reverse_acc = 1.0;
      const double max_acc_jerk = 0.5;
      const double delta_t = 0.2;

      SpeedData speed_data;

      // TODO(Jinyun): explore better time horizon heuristic
      const double path_length = result->accumulated_s.back();
      const double total_t = std::max(gear ? 1.5 *
                                                 (max_forward_v * max_forward_v +
                                                  path_length * max_forward_acc) /
                                                 (max_forward_acc * max_forward_v)
                                           : 1.5 *
                                                 (max_reverse_v * max_reverse_v +
                                                  path_length * max_reverse_acc) /
                                                 (max_reverse_acc * max_reverse_v),
                                      10.0);

      const size_t num_of_knots = static_cast<size_t>(total_t / delta_t) + 1;

      PiecewiseJerkSpeedProblem piecewise_jerk_problem(
          num_of_knots, delta_t, {0.0, std::abs(init_v), std::abs(init_a)});

      // set end constraints
      std::vector<std::pair<double, double>> x_bounds(num_of_knots,
                                                      {0.0, path_length});

      const double max_v = gear ? max_forward_v : max_reverse_v;
      const double max_acc = gear ? max_forward_acc : max_reverse_acc;

      const auto upper_dx = std::fmax(max_v, std::abs(init_v));
      std::vector<std::pair<double, double>> dx_bounds(num_of_knots,
                                                       {0.0, upper_dx});
      std::vector<std::pair<double, double>> ddx_bounds(num_of_knots,
                                                        {-max_acc, max_acc});

      x_bounds[num_of_knots - 1] = std::make_pair(path_length, path_length);
      dx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);
      ddx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);

      // TODO(Jinyun): move to confs
      std::vector<double> x_ref(num_of_knots, path_length);
      piecewise_jerk_problem.set_x_ref(10000.0, std::move(x_ref));
      piecewise_jerk_problem.set_weight_ddx(10.0);
      piecewise_jerk_problem.set_weight_dddx(10.0);
      piecewise_jerk_problem.set_x_bounds(std::move(x_bounds));
      piecewise_jerk_problem.set_dx_bounds(std::move(dx_bounds));
      piecewise_jerk_problem.set_ddx_bounds(std::move(ddx_bounds));
      piecewise_jerk_problem.set_dddx_bound(max_acc_jerk);

      // solve the problem
      if (!piecewise_jerk_problem.Optimize())
      {
        AERROR << "Piecewise jerk speed optimizer failed!";
        return false;
      }

      // extract output
      const std::vector<double> &s = piecewise_jerk_problem.opt_x();
      const std::vector<double> &ds = piecewise_jerk_problem.opt_dx();
      const std::vector<double> &dds = piecewise_jerk_problem.opt_ddx();

      // assign speed point by gear
      speed_data.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
      const double kEpislon = 1.0e-6;
      const double sEpislon = 1.0e-6;
      for (size_t i = 1; i < num_of_knots; ++i)
      {
        if (s[i - 1] - s[i] > kEpislon)
        {
          ADEBUG << "unexpected decreasing s in speed smoothing at time "
                 << static_cast<double>(i) * delta_t << "with total time "
                 << total_t;
          break;
        }
        speed_data.AppendSpeedPoint(s[i], delta_t * static_cast<double>(i), ds[i],
                                    dds[i], (dds[i] - dds[i - 1]) / delta_t);
        // cut the speed data when it is about to meet end condition
        if (path_length - s[i] < sEpislon)
        {
          break;
        }
      }

      // combine speed and path profile
      DiscretizedPath path_data;
      for (size_t i = 0; i < path_points_size; ++i)
      {
        common::PathPoint path_point;
        path_point.set_x(result->x[i]);
        path_point.set_y(result->y[i]);
        path_point.set_theta(result->phi[i]);
        path_point.set_s(result->accumulated_s[i]);
        path_data.push_back(std::move(path_point));
      }

      HybridAStartResult combined_result;

      // TODO(Jinyun): move to confs
      const double kDenseTimeResoltuion = 0.5;
      const double time_horizon =
          speed_data.TotalTime() + kDenseTimeResoltuion * 1.0e-6;
      if (path_data.empty())
      {
        AERROR << "path data is empty";
        return false;
      }
      for (double cur_rel_time = 0.0; cur_rel_time < time_horizon;
           cur_rel_time += kDenseTimeResoltuion)
      {
        common::SpeedPoint speed_point;
        if (!speed_data.EvaluateByTime(cur_rel_time, &speed_point))
        {
          AERROR << "Fail to get speed point with relative time " << cur_rel_time;
          return false;
        }

        if (speed_point.s() > path_data.Length())
        {
          break;
        }

        common::PathPoint path_point = path_data.Evaluate(speed_point.s());

        combined_result.x.push_back(path_point.x());
        combined_result.y.push_back(path_point.y());
        combined_result.phi.push_back(path_point.theta());
        combined_result.accumulated_s.push_back(path_point.s());
        if (!gear)
        {
          combined_result.v.push_back(-speed_point.v());
          combined_result.a.push_back(-speed_point.a());
        }
        else
        {
          combined_result.v.push_back(speed_point.v());
          combined_result.a.push_back(speed_point.a());
        }
      }

      combined_result.a.pop_back();

      // recalc step size
      path_points_size = combined_result.x.size();

      // load steering from phi
      for (size_t i = 0; i + 1 < path_points_size; ++i)
      {
        double discrete_steer =
            (combined_result.phi[i + 1] - combined_result.phi[i]) *
            vehicle_param_.wheel_base() /
            (combined_result.accumulated_s[i + 1] -
             combined_result.accumulated_s[i]);
        discrete_steer =
            gear ? std::atan(discrete_steer) : std::atan(-discrete_steer);
        combined_result.steer.push_back(discrete_steer);
      }

      *result = combined_result;
      return true;
    }

    bool HybridAStar::TrajectoryPartition(
        const HybridAStartResult &result,
        std::vector<HybridAStartResult> *partitioned_result)
    {
      const auto &x = result.x;
      const auto &y = result.y;
      const auto &phi = result.phi;
      if (x.size() != y.size() || x.size() != phi.size())
      {
        AERROR << "states sizes are not equal when do trajectory partitioning of "
                  "Hybrid A Star result";
        return false;
      }

      size_t horizon = x.size();
      partitioned_result->clear();
      partitioned_result->emplace_back();
      auto *current_traj = &(partitioned_result->back());
      double heading_angle = phi.front();
      const Vec2d init_tracking_vector(x[1] - x[0], y[1] - y[0]);
      double tracking_angle = init_tracking_vector.Angle();
      bool current_gear =
          std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
          (M_PI_2);
      for (size_t i = 0; i < horizon - 1; ++i)
      {
        heading_angle = phi[i];
        const Vec2d tracking_vector(x[i + 1] - x[i], y[i + 1] - y[i]);
        tracking_angle = tracking_vector.Angle();
        bool gear =
            std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
            (M_PI_2);
        if (gear != current_gear)
        {
          current_traj->x.push_back(x[i]);
          current_traj->y.push_back(y[i]);
          current_traj->phi.push_back(phi[i]);
          partitioned_result->emplace_back();
          current_traj = &(partitioned_result->back());
          current_gear = gear;
        }
        current_traj->x.push_back(x[i]);
        current_traj->y.push_back(y[i]);
        current_traj->phi.push_back(phi[i]);
      }
      current_traj->x.push_back(x.back());
      current_traj->y.push_back(y.back());
      current_traj->phi.push_back(phi.back());

      const auto start_timestamp = std::chrono::system_clock::now();

      // Retrieve v, a and steer from path
      for (auto &result : *partitioned_result)
      {
        if (FLAGS_use_s_curve_speed_smooth)
        {
          if (!GenerateSCurveSpeedAcceleration(&result))
          {
            AERROR << "GenerateSCurveSpeedAcceleration fail";
            return false;
          }
        }
        else
        {
          if (!GenerateSpeedAcceleration(&result))
          {
            AERROR << "GenerateSpeedAcceleration fail";
            return false;
          }
        }
      }

      const auto end_timestamp = std::chrono::system_clock::now();
      std::chrono::duration<double> diff = end_timestamp - start_timestamp;
      ADEBUG << "speed profile total time: " << diff.count() * 1000.0 << " ms.";
      return true;
    }

    bool HybridAStar::GetTemporalProfile(HybridAStartResult *result)
    {
      std::vector<HybridAStartResult> partitioned_results;
      if (!TrajectoryPartition(*result, &partitioned_results))
      {
        AERROR << "TrajectoryPartition fail";
        return false;
      }
      HybridAStartResult stitched_result;
      for (const auto &result : partitioned_results)
      {
        std::copy(result.x.begin(), result.x.end() - 1,
                  std::back_inserter(stitched_result.x));
        std::copy(result.y.begin(), result.y.end() - 1,
                  std::back_inserter(stitched_result.y));
        std::copy(result.phi.begin(), result.phi.end() - 1,
                  std::back_inserter(stitched_result.phi));
        std::copy(result.v.begin(), result.v.end() - 1,
                  std::back_inserter(stitched_result.v));
        std::copy(result.a.begin(), result.a.end(),
                  std::back_inserter(stitched_result.a));
        std::copy(result.steer.begin(), result.steer.end(),
                  std::back_inserter(stitched_result.steer));
      }
      stitched_result.x.push_back(partitioned_results.back().x.back());
      stitched_result.y.push_back(partitioned_results.back().y.back());
      stitched_result.phi.push_back(partitioned_results.back().phi.back());
      stitched_result.v.push_back(partitioned_results.back().v.back());
      *result = stitched_result;
      return true;
    }
    // A*的主要流程
    bool HybridAStar::Plan(
        double sx, double sy, double sphi, double ex, double ey, double ephi,        //起点、终点
        const std::vector<double> &XYbounds,                                         //地图？？
        const std::vector<std::vector<common::math::Vec2d>> &obstacles_vertices_vec, //障碍物顶点信息
        HybridAStartResult *result)
    { //结果
      // clear containers1.初始化清空
      open_set_.clear();               //无序图数据结构 开集 记录了所有待遍历和遍历过的节点
      close_set_.clear();              //闭集 记录了所有已经遍历过的节点
      open_pq_ = decltype(open_pq_)(); //队列根据节点cost由小到达的顺序排列
      final_node_ = nullptr;

      // 2.构造障碍物轮廓线段。为什么不把这一段代码封装在obstacle类中
      std::vector<std::vector<common::math::LineSegment2d>>
          obstacles_linesegments_vec; //障碍物线段数组
      for (const auto &obstacle_vertices : obstacles_vertices_vec)
      {
        size_t vertices_num = obstacle_vertices.size();
        std::vector<common::math::LineSegment2d> obstacle_linesegments;
        //网友反馈：我认为这里有错，少构造了一条线段。等待校验
        //(obstacle_vertices[vertices_num - 1], obstacle_vertices[0]
        for (size_t i = 0; i < vertices_num - 1; ++i)
        {
          common::math::LineSegment2d line_segment = common::math::LineSegment2d(
              obstacle_vertices[i], obstacle_vertices[i + 1]);
          obstacle_linesegments.emplace_back(line_segment); //尾插
        }
        obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
      }
      obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);

      // load XYbounds
      // 3.载入起点终点并做碰撞校验
      XYbounds_ = XYbounds;
      // load nodes and obstacles
      start_node_.reset(
          new Node3d({sx}, {sy}, {sphi}, XYbounds_, planner_open_space_config_));
      end_node_.reset(
          new Node3d({ex}, {ey}, {ephi}, XYbounds_, planner_open_space_config_));
      if (!ValidityCheck(start_node_))
      {
        AERROR << "start_node in collision with obstacles";
        return false;
      }
      if (!ValidityCheck(end_node_))
      {
        AERROR << "end_node in collision with obstacles";
        return false;
      }
      // 4.生成动态规划代价地图，以终点为搜索起点，遍历整个地图计算各个点到终点的代价。
      double map_time = Clock::NowInSeconds();

      grid_a_star_heuristic_generator_->GenerateDpMap(ex, ey, XYbounds_,
                                                      obstacles_linesegments_vec_);
      ADEBUG << "map time " << Clock::NowInSeconds() - map_time;
      // load open set, pq5.载入起点至open_set、open_pq
      open_set_.emplace(start_node_->GetIndex(), start_node_);
      open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());

      // Hybrid A* begins
      size_t explored_node_num = 0;
      double astar_start_time = Clock::NowInSeconds();
      double heuristic_time = 0.0;
      double rs_time = 0.0;

      while (!open_pq_.empty())
      {
        // take out the lowest cost neighboring node
        const std::string current_id = open_pq_.top().first;          //取代价最小的节点
        open_pq_.pop();                                               //从队列中取出最小值
        std::shared_ptr<Node3d> current_node = open_set_[current_id]; //存储当前节点
        // check if an analystic curve could be connected from current
        // configuration to the end configuration without collision. if so, search
        // ends.如果有一条RS曲线连接当前点和终点无碰撞，那么终止搜索
        const double rs_start_time = Clock::NowInSeconds();
        if (AnalyticExpansion(current_node))
        {
          break;
        }
        const double rs_end_time = Clock::NowInSeconds();
        rs_time += rs_end_time - rs_start_time;
        //将当前节点放入close_set
        close_set_.emplace(current_node->GetIndex(), current_node);
        //遍历邻接节点,并计算cost
        for (size_t i = 0; i < next_node_num_; ++i)
        { // 3个方向扩展
          //以不同方向，向前行驶根号2倍的栅格分辨率向前行进
          std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
          // boundary check failure handle
          if (next_node == nullptr)
          {
            continue; //超出边界，跳过
          }
          // check if the node is already in the close set检测是否已经在close_set
          if (close_set_.find(next_node->GetIndex()) != close_set_.end())
          {
            continue; //已经检测到跳过
          }
          // collision check碰撞检测
          if (!ValidityCheck(next_node))
          {
            continue; //跳出本次循环
          }
          if (open_set_.find(next_node->GetIndex()) == open_set_.end())
          {
            explored_node_num++;
            const double start_time = Clock::NowInSeconds();
            CalculateNodeCost(current_node, next_node); //计算节点cost
            const double end_time = Clock::NowInSeconds();
            heuristic_time += end_time - start_time;
            open_set_.emplace(next_node->GetIndex(), next_node);           //邻接节点放入open_set
            open_pq_.emplace(next_node->GetIndex(), next_node->GetCost()); //邻接节点放入open_pq优先队列
          }
        }
      }
      if (final_node_ == nullptr)
      {
        ADEBUG << "Hybrid A searching return null ptr(open_set ran out)";
        return false;
      }
      if (!GetResult(result))
      {
        ADEBUG << "GetResult failed";
        return false;
      }
      ADEBUG << "explored node num is " << explored_node_num;
      ADEBUG << "heuristic time is " << heuristic_time;
      ADEBUG << "reed shepp time is " << rs_time;
      ADEBUG << "hybrid astar total time is "
             << Clock::NowInSeconds() - astar_start_time;
      return true;
    }
  } // namespace planning
} // namespace apollo
