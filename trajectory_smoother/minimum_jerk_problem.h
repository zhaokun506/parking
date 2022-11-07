#if 0

#include "OsqpEigen/OsqpEigen.h"
#include "configs/planner_open_space_config.h"
#include "configs/vehicle_config.h"
#include "eigen3/Eigen/Dense"
class MinimumJerkProblem {

public:
  MinimumJerkProblem(const PlannerOpenSpaceConfig &planner_open_space_config,
                     const VehicleParam vehicle_param);
  ~MinimumJerkProblem();
  /*
  路标点矩阵
  */
  bool Solve(const Eigen::MatrixXd &way_points, //路经点
             Eigen::MatrixXd *state_result,     //最终平滑的轨迹
             Eigen::MatrixXd *control_result);  //最终轨迹对应的控制量

private:
  Eigen::VectorXd TimeAllocation(const Eigen::MatrixXd &Path);

  Eigen::MatrixXd PolyQPGeneration(
      const int d_order,           // the order of derivative
      const Eigen::MatrixXd &Path, // waypoints coordinates (3d or 2D)
      const Eigen::MatrixXd &Vel,  // boundary velocity
      const Eigen::MatrixXd &Acc,  // boundary acceleration
      const Eigen::VectorXd &Time, // time allocation in each segment
      OsqpEigen::Solver &slover);

  //代价函数的hession矩阵，P_t*Q*P
  void GetHession(const int n_seg, const int d_order,
                  const Eigen::VectorXd &Time,
                  Eigen::SparseMatrix<double> &hession);
  void GetLinearConstraintsMatrix(const int n_seg, const int d_order,
                                  const Eigen::VectorXd &Time,
                                  Eigen::SparseMatrix<double> &linearMatrix);
  void InsertCoff(const int row, const int col,
                  Eigen::SparseMatrix<double> &linearMatrix, const double t,
                  const int d_order, bool one_line, bool reverse);

  int MinimumJerkProblem::Factorial(int x);

  void GetResult(const Eigen::MatrixXd poly_coeff, const Eigen::VectorXd time,
                 const int d_order, const int delta_t,
                 Eigen::VectorXd *time_result, Eigen::MatrixXd *state_result,
                 Eigen::MatrixXd *control_result);

private:
  const PlannerOpenSpaceConfig &planner_open_space_config_;
  const VehicleParam vehicle_param_;
};

#endif