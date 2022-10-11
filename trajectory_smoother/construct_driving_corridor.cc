#include "trajectory_smoother/construct_driving_corridor.h"

ConstructDrivingCorridor::ConstructDrivingCorridor(
    const PlannerOpenSpaceConfig &open_space_config,
    VehicleParam &vehicle_config)
    : open_space_config_(open_space_config), vehicle_config_(vehicle_config) {
} //应该进行相关的初始化工作

// ConstructDrivingCorridor::ConstructDrivingCorridor(
//     PlannerOpenSpaceConfig &open_space_config, VehicleParam &vehicle_config)
//     {}

int ConstructDrivingCorridor::Construct(
    const std::vector<std::vector<common::math::Vec2d>>
        &obstacles_vertices_vec, //障碍物顶点数组，顶点的表示法为Vec2d向量,
    const Eigen::MatrixXd &xWs, //初始路径
    const int &n,               //离散点数
    Eigen::MatrixXd
        *f_bound, // 1.车辆前圆心可行驶边界n*4矩阵,n*(x_min,x_max,y_min,y_max)
    Eigen::MatrixXd *r_bound) {
  //膨胀障碍物
  std::vector<std::vector<common::math::Vec2d>> swelling_obstacles_vec;
  for (const auto &obs : obstacles_vertices_vec) {
    std::vector<common::math::Vec2d> swelling_obstacles_vertices;
    SwellingObstacles(obs, 2, &swelling_obstacles_vertices); //膨胀障碍物
    swelling_obstacles_vec.push_back(swelling_obstacles_vertices);
  }

  //路径点重采样？？是否需要，暂且不写

  //由计算前后等效圆心
  int size = xWs.cols();
  for (int i = 0; i < size; i++) {
    double x = xWs(i, 0);
    double y = xWs(i, 1);
    double theta = xWs(i, 2);
    //根据车辆中心计算前后覆盖圆中心
    double x_f = x + (3 / 4 * vehicle_config_.length -
                      vehicle_config_.back_edge_to_center) *
                         cos(theta);
    double y_f = y + (3 / 4 * vehicle_config_.length -
                      vehicle_config_.back_edge_to_center) *
                         sin(theta);
    double x_b = x + (1 / 4 * vehicle_config_.length -
                      vehicle_config_.back_edge_to_center) *
                         cos(theta);
    double y_b = y + (1 / 4 * vehicle_config_.length -
                      vehicle_config_.back_edge_to_center) *
                         sin(theta);
    double ds = 0.1;
    Eigen::MatrixXd driving_bound_fbox(1, 4); //(x_max,x_min,y_max,y_min)
    GenerateDrivingBoundBox(swelling_obstacles_vec, x_f, y_f, ds, 5,
                            &driving_bound_fbox);
    Eigen::MatrixXd driving_bound_bbox(1, 4);
    GenerateDrivingBoundBox(swelling_obstacles_vec, x_b, y_b, ds, 5,
                            &driving_bound_bbox);

    (*f_bound).block(i, 0, 1, 4) = driving_bound_fbox;
    (*r_bound).block(i, 0, 1, 4) = driving_bound_bbox;
  }

  //生产局部矩形
}

//对障碍物进行膨胀处理
void ConstructDrivingCorridor::SwellingObstacles(
    const std::vector<common::math::Vec2d> &obstacles_vertices, const double &r,
    std::vector<common::math::Vec2d> *swelling_obstacles_vertices) {
  std::vector<common::math::Vec2d> temp_obstacles_vertices = obstacles_vertices;
  //将第一个点放在末尾，将末尾点放在头部使每个点都有相邻点
  common::math::Vec2d first_vertice = obstacles_vertices.front();
  common::math::Vec2d last_vertice = obstacles_vertices.back();
  temp_obstacles_vertices.insert(temp_obstacles_vertices.begin(), last_vertice);
  temp_obstacles_vertices.push_back(first_vertice);

  int size = temp_obstacles_vertices.size();
  int idx = 0;
  for (int i = 1; i < size - 2; i++) {
    common::math::Vec2d left_cur =
        (temp_obstacles_vertices[i] - temp_obstacles_vertices[i - 1]) /
        left_cur.Length(); // left->cur单位矢量
    common::math::Vec2d right_cur =
        (temp_obstacles_vertices[i] - temp_obstacles_vertices[i + 1]) /
        right_cur.Length(); // right->cur单位矢量
    double sin_theta = left_cur.CrossProd(right_cur);
    double d_cur_swell = 0; //计算sinθ
    if (sin_theta == 0) {
      std::cout << "障碍物顶点错误..." << std::endl;
      continue;
    } else {
      d_cur_swell = r / sin_theta;
    }
    common::math::Vec2d cur_swel =
        d_cur_swell * (left_cur + right_cur); //膨胀方向,变成单位相邻*d

    (*swelling_obstacles_vertices)[idx] = temp_obstacles_vertices[i] + cur_swel;
    idx++;
  }
}
//生成可行驶区域矩形
void ConstructDrivingCorridor::GenerateDrivingBoundBox(
    const std::vector<std::vector<common::math::Vec2d>> &swelling_obstacles,
    const double x, const double y, const double ds, const double l_limit,
    Eigen::MatrixXd *driving_bound_box) {
  std::vector<double> bound = {ds, 0, 0, 0}; //{x_max,y_max,x_min,y_min}

  std::vector<int> dirs = {1, 2, 3, 0};
  while (!dirs.empty()) //如果还有方向没有搜索完成，则一直搜索
  {
    for (int i = 0; i < dirs.size(); i++) {

      bound[dirs[i]] = bound[dirs[i]] + ds; //向该方向扩展ds

      common::math::Vec2d right_up(x + bound[0], y + bound[1]);
      common::math::Vec2d right_low(x + bound[0], y + bound[3]);
      common::math::Vec2d left_low(x + bound[2], y + bound[3]);
      common::math::Vec2d left_up(x + bound[2], y + bound[1]);
      std::vector<common::math::Vec2d> box;
      box.push_back(right_up);
      box.push_back(right_low);
      box.push_back(left_low);
      box.push_back(left_up);
      //遍历所有障碍物检测是否碰撞
      bool is_collision;
      for (const auto &obs : swelling_obstacles)
        CollisionCheck(box, obs, &is_collision); //暂时不考虑障碍物检测错误

      if (is_collision ||
          bound[dirs[i]] > l_limit) //如果发生碰撞或者达到边界限制
      {
        dirs.erase(dirs.begin() + i); //探索完成则从方向列表删除
      }
    }
  }
  (*driving_bound_box)(1, 0) = bound[0];
  (*driving_bound_box)(1, 1) = bound[2];
  (*driving_bound_box)(1, 2) = bound[1];
  (*driving_bound_box)(1, 3) = bound[3];
}

/*障碍物检测的方法有
    1.外接圆
    2.AABB 与坐标值平行的包围盒
    3.SAT 分离轴
    4.OBB
    5.GJK  EPA
    6.比较面积法
*/
int ConstructDrivingCorridor::CollisionCheck(
    const std::vector<common::math::Vec2d> &driving_box_vertices,
    const std::vector<common::math::Vec2d> &convex_polygon_vertices,
    bool *result) {
  if (driving_box_vertices.size() != 4 || convex_polygon_vertices.size() < 3) {
    std::cout << "输入错误:可行框不是4个顶点,障碍物少于3个顶点" << std::endl;
    return 0;
  } else {
    //首先采用 AABB进行粗略判断，然后再使用SAT进行精细判断
    std::vector<double> bounding_box_driving;
    std::vector<double> bounding_box_obs;
    BuildBoundingBox(driving_box_vertices, &bounding_box_driving);
    BuildBoundingBox(convex_polygon_vertices, &bounding_box_obs);

    //在x或y轴无重叠，则无碰撞
    bool xy_overlap = (bounding_box_driving[0] < bounding_box_obs[2] ||
                       bounding_box_driving[2] > bounding_box_obs[0]) ||
                      (bounding_box_driving[1] < bounding_box_obs[3] ||
                       bounding_box_driving[3] > bounding_box_obs[1]);
    if (xy_overlap) {
      (*result) = false;
    } else //如果AABB碰撞，则进行精确碰撞检测
    {
      std::vector<common::math::Vec2d> project_axis_vec;
      common::math::Vec2d x_axis(0, 1);
      common::math::Vec2d y_axis(1, 0);
      project_axis_vec.push_back(x_axis);
      project_axis_vec.push_back(y_axis);

      //生成投影轴
      for (int i = 0; i < convex_polygon_vertices.size() - 1; i++) {
        common::math::Vec2d vec_edge;
        if (i == 0)
          vec_edge =
              convex_polygon_vertices[i] - convex_polygon_vertices.back();
        else
          vec_edge =
              convex_polygon_vertices[i] - convex_polygon_vertices[i - 1];
        vec_edge = vec_edge / vec_edge.Length();
        common::math::Vec2d vec_edge_n(vec_edge.y(), -vec_edge.x());

        if (!project_axis_vec.empty()) {
          bool is_same = false;
          for (const auto &axis : project_axis_vec) {
            if (vec_edge_n.CrossProd(axis) == 0) {
              is_same = true;
              break;
            }
          }
          if (!is_same) {
            project_axis_vec.push_back(vec_edge_n);
          }
        } else {
          project_axis_vec.push_back(vec_edge_n);
        }
      }

      for (const auto &project_axis : project_axis_vec) {
        //此处可以用优先队列
        //大顶推满足的条件是每一个父节点都比子节点大
        // auto cmp = [&](const common::math::Vec2d vec1, const
        // common::math::Vec2d vec2) { return vec1.x() > vec2.x(); };
        // //大顶堆，子节点与父节点的关系
        //                                  //
        //                                  std::priority_queue<common::math::Vec2d,
        //                                  std::vector<common::math::Vec2d>,
        //                                  decltype(cmp)> box_projects;
        //                                  //
        //                                  std::priority_queue<common::math::Vec2d,
        //                                  std::vector<common::math::Vec2d>,
        //                                  decltype(cmp)> obs_projects;
        std::vector<double> box_projects;
        std::vector<double> obs_projects;
        double box_x_max = -DBL_MAX, box_x_min = DBL_MAX, obs_x_max = -DBL_MAX,
               obs_x_min = DBL_MAX;

        for (const auto &driving_box_vertice :
             driving_box_vertices) //将可行驶框投影到投影轴，并按照x排序
        {
          double project = project_axis.InnerProd(driving_box_vertice);
          box_projects.push_back(project);
          if (project > box_x_max)
            box_x_max = project;
          if (project < box_x_min)
            box_x_min = project;
        }

        for (const auto &convex_polygon_vertice :
             driving_box_vertices) //将障碍物投影到投影轴
        {
          double project = project_axis.InnerProd(convex_polygon_vertice);
          obs_projects.push_back(project);
          if (project > obs_x_max)
            obs_x_max = project;
          if (project < obs_x_min)
            obs_x_min = project;
        }

        if (box_x_max < obs_x_min ||
            box_x_min >
                obs_x_max) { //如果存在一个投影轴，box和obs的投影不想交，则无碰撞
          (*result) = false;
          break;
        }
      }
    }

    return 1;
  }
}
void ConstructDrivingCorridor::BuildBoundingBox(
    const std::vector<common::math::Vec2d> &polygon_vertices,
    std::vector<double> *bounding_box) { //{x_max,y_max,x_min,y_min}
  for (const auto &vec : polygon_vertices) {
    if (vec.x() > (*bounding_box)[0]) {
      (*bounding_box)[0] = vec.x();
    }
    if (vec.x() < (*bounding_box)[2]) {

      (*bounding_box)[2] = vec.x();
    }
    if (vec.y() > (*bounding_box)[1]) {

      (*bounding_box)[1] = vec.y();
    }
    if (vec.y() < (*bounding_box)[3]) {

      (*bounding_box)[3] = vec.y();
    }
  }
}
