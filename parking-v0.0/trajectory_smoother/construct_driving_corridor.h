/*该类用于行车隧道的生成，以便将障碍物非线性非凸的约束转换为线性的约束


*/

/*
一个主逻辑类+几个子模块类，实现一定逻辑的功能。

主逻辑类，私有成员包含其他几个子模块的指针。关联关系。
主逻辑类还应将子模块及穿入参数的数据结构转换为通用的数据结构，已减少模块间的相互依赖。
主逻辑类的主函数实现了逻辑运行。
 */
#include <iostream>
#include <vector>
#include "planner_open_space_config.h"
#include "Eigen/Eigen"
#include "Eigen/Dense"
#include "common/math/vec2d.h"

class ConstructDrivingCorridor
{
    ConstructDrivingCorridor(PlannerOpenSpaceConfig &open_space_config); //应该进行相关的初始化工作
    ~ConstructDrivingCorridor() = default;

public:
    //

    //类的主函数，，返回值应该为status。此处用int代替，1代表0K，0代表错误
    //输入：1.地图中所有障碍物的信息 2.初始轨迹，3.离散点数,
    //输出,将车辆等效为两个圆盘模型1.车辆前圆心可行驶边界n*4矩阵,n*(x_min,x_max,y_min,y_max)
    int Construct(const std::vector<std::vector<common::math::Vec2d>> &obstacles_vertices_vec, //障碍物顶点数组，顶点的表示法为Vec2d向量,
                  const Eigen::MatrixXd *xWs,                                                  //初始路径的
                  const int &n,                                                                //离散点数
                  Eigen::MatrixXd *f_bound,                                                    // 1.车辆前圆心可行驶边界n*4矩阵,n*(x_min,x_max,y_min,y_max)
                  Eigen::MatrixXd *r_bound);

    //对障碍物进行膨胀处理
    void SwellingObstacles(const std::vector<common::math::Vec2d> &obstacles_vertices,
                           const double &r,
                           std::vector<common::math::Vec2d> *swelling_obstacles_vertices);

    void GenerateRectangleBound(const std::vector<std::vector<common::math::Vec2d>> &swelling_obstacles,
                                const double x, const double y,
                                const double delta_s, const double l_limit,
                                Eigen::MatrixXd *rectangle);
    //障碍物以什么样的形式存在呢，点，边？？

    //由 点、障碍物，生成约束矩形
};