#include "common/math/vec2d.h"
#include "eigen3/Eigen/Eigen"
#include <iostream>
class OpenSpaceRoiDecider {
private:
  /* data */
public:
  OpenSpaceRoiDecider(/* args */);
  ~OpenSpaceRoiDecider();

  bool GetHyperPlanes(const int &obstacles_num,
                      const Eigen::MatrixXi &obstacles_edges_num,
                      const std::vector<std::vector<common::math::Vec2d>>
                          &obstacles_vertices_vec,
                      Eigen::MatrixXd *A_all, Eigen::MatrixXd *b_all);
};
