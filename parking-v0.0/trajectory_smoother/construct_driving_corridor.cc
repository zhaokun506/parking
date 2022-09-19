#include "trajectory_smoother/construct_driving_corridor.h"

int ConstructDrivingCorridor::Construct(const std::vector<std::vector<common::math::Vec2d>> &obstacles_vertices_vec, //障碍物顶点数组，顶点的表示法为Vec2d向量,
                                        const Eigen::MatrixXd *xWs,                                                  //初始路径
                                        const int &n,                                                                //离散点数
                                        Eigen::MatrixXd *f_bound,                                                    // 1.车辆前圆心可行驶边界n*4矩阵,n*(x_min,x_max,y_min,y_max)
                                        Eigen::MatrixXd *r_bound)
{
    std::vector<common::math::Vec2d> swelling_obstacles_vertices;
    SwellingObstacles(obstacles_vertices_vec, 2, &swelling_obstacles_vertices);
    //路径点重采样？？是否需要，暂且不写

    //由计算前后等效圆心

    //生产局部矩形
}

//对障碍物进行膨胀处理
void ConstructDrivingCorridor::SwellingObstacles(const std::vector<common::math::Vec2d> &obstacles_vertices,
                                                 const double &r,
                                                 std::vector<common::math::Vec2d> *swelling_obstacles_vertices)
{
    std::vector<common::math::Vec2d> temp_obstacles_vertices = obstacles_vertices;
    //将第一个点放在末尾，将末尾点放在头部使每个点都有相邻点
    common::math::Vec2d first_vertice = obstacles_vertices.front();
    common::math::Vec2d last_vertice = obstacles_vertices.back();
    temp_obstacles_vertices.insert(temp_obstacles_vertices.begin(), last_vertice);
    temp_obstacles_vertices.push_back(first_vertice);

    int size = temp_obstacles_vertices.size();
    int idx = 0;
    for (int i = 1; i < size - 2; i++)
    {
        common::math::Vec2d left_cur = (temp_obstacles_vertices[i] - temp_obstacles_vertices[i - 1]) / left_cur.Length();   // left->cur单位矢量
        common::math::Vec2d right_cur = (temp_obstacles_vertices[i] - temp_obstacles_vertices[i + 1]) / right_cur.Length(); // right->cur单位矢量
        double sin_theta = left_cur.CrossProd(right_cur);
        double d_cur_swell = 0; //计算sinθ
        if (sin_theta == 0)
        {
            std::cout << "障碍物顶点错误..." << std::endl;
            continue;
        }
        else
        {
            d_cur_swell = r / sin_theta;
        }
        common::math::Vec2d cur_swel = d_cur_swell * (left_cur + right_cur); //膨胀方向,变成单位相邻*d

        (*swelling_obstacles_vertices)[idx] = temp_obstacles_vertices[i] + cur_swel;
        idx++;
    }
}

void ConstructDrivingCorridor::GenerateRectangleBound(const std::vector<std::vector<common::math::Vec2d>> &swelling_obstacles,
                                                      const double x, const double y,
                                                      const double ds, const double l_limit,
                                                      Eigen::MatrixXd *rectangle)
{
    std::vector<double> d_bound = {ds, 0, 0, 0}; //{x_max,y_max,x_min,y_min}
    std::vector<double> bounnd

        std::vetcor<int>
            dirs = {1, 2, 3, 0};

    while (!dirs.empty()) //如果还有方向没有搜索完成，则一直搜索
    {
        for (const auto dir : dirs)
        {
            d_bound[dir] = d_bound[dir] + ds; //向该方向扩展

            common::math::Vec2d right_up
        }
    }
}