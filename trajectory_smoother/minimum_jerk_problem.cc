#if 0
#include "minimum_jerk_problem.h"

MinimumJerkProblem::MinimumJerkProblem(
    const PlannerOpenSpaceConfig &planner_open_space_config,
    const VehicleParam vehicle_param)
    : planner_open_space_config_(planner_open_space_config),
      vehicle_param_(vehicle_param) {}

bool MinimumJerkProblem::Solve(
    const Eigen::MatrixXd &way_points, // path的way_point表示(x,y,z)*n行
    Eigen::MatrixXd *state_result,     //状态量结果（x,y）
    Eigen::MatrixXd *control_result) { //控制量结果

  auto time = TimeAllocation(way_points); //时间分配

  //最优化求多项式系数
  // boundary velocity 其实点速度，加速度
  const Eigen::MatrixXd vel_bound = Eigen::MatrixXd::Zero(2, 3);
  // boundary acceleration
  const Eigen::MatrixXd acc_bound = Eigen::MatrixXd::Zero(2, 3);
  OsqpEigen::Solver slover;
  auto poly_coeff =
      PolyQPGeneration(3, way_points, time, vel_bound, acc_bound, slover);
}

Eigen::MatrixXd MinimumJerkProblem::PolyQPGeneration(
    const int d_order,           // the order of derivative，jerk-3，snap-4
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d，or 2d)
    const Eigen::MatrixXd &vel_bound, // boundary velocity
    const Eigen::MatrixXd &acc_bound, // boundary acceleration
    const Eigen::VectorXd &Time,      // time allocation in each segment
    OsqpEigen::Solver &slover) {
  // enforce initial and final velocity and accleration, for higher order
  // derivatives, just assume them be 0;
  int p_order = 2 * d_order - 1; // the order of polynomial,多项式阶数
  // the number of variables in each segment多项式系数个数
  int p_num1d = p_order + 1;
  // the number of segments分段的数量
  int m = Time.size();
  // position(x,y,z), so we need (3 * p_num1d) coefficients,多项式系数矩阵
  Eigen::MatrixXd PolyCoeff = Eigen::MatrixXd::Zero(m, 3 * p_num1d);
  Eigen::VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

  /**
   * osqp
   * minimize 1/2 * x'P x+q'*x
   * s.t        l<= Ax <=u
   * A(m*n)*x(n*1)=constraint(m*1)
   *
   */

  // slover.settings()->setWarmStart(true);

  // 设置变量个数,段数*系数个数
  slover.data()->setNumberOfVariables(m * p_num1d);

  //设置约束个数，即A矩阵的列数。起始点约束2*3，路标点约束1*m-1,连续性约束3*m-1
  slover.data()->setNumberOfConstraints(d_order * 2 + (m - 1) * (d_order + 1));

  // 设置H矩阵
  Eigen::SparseMatrix<double> hession;
  GetHession(m, d_order, Time, hession);
  if (!slover.data()->setHessianMatrix(hession)) {
    std::cout << "设置hession矩阵失败";
    return Eigen::MatrixXd::Zero(1, 1);
  } else {
    std::cout << "hession矩阵设置成功" << std::endl;
  }

  //设置线性约束矩阵
  Eigen::SparseMatrix<double> linearMatrix;
  GetLinearConstraintsMatrix(m, d_order, Time, linearMatrix);

  if (!slover.data()->setLinearConstraintsMatrix(linearMatrix)) {
    std::cout << "设置Linear矩阵失败";
    return Eigen::MatrixXd::Zero(1, 1);
  } else {
    std::cout << "Linear矩阵设置成功" << std::endl;
  };

  Eigen::VectorXd gradient(p_num1d * m);
  gradient.setZero();

  // 设置梯度约束
  if (!slover.data()->setGradient(gradient)) {
    std::cout << "梯度设置失败" << std::endl;
  } else {
    std::cout << "梯度设置成功" << std::endl;
  }

  // 设置边界，求解问题
  Eigen::VectorXd lowbound =
      Eigen::VectorXd::Zero(d_order * 2 + (m - 1) * (d_order + 1));
  Eigen::VectorXd upbound =
      Eigen::VectorXd::Zero(d_order * 2 + (m - 1) * (d_order + 1));

  slover.data()->setLowerBound(lowbound);
  slover.data()->setUpperBound(upbound);

  //初始化求解器
  if (!slover.isInitialized()) {
    slover.initSolver();
  }

  for (int dim = 0; dim < 3; dim++) {

    Eigen::VectorXd wayPoints = Path.col(dim);

    // 起点位置
    lowbound(0) = wayPoints(0);
    upbound(0) = wayPoints(0);

    // 终点位置
    lowbound(d_order) = wayPoints(m);
    upbound(d_order) = wayPoints(m);

    // 固定中间节点位置
    for (int i = 0; i < m - 1; i++) {
      lowbound(2 * d_order + i) = wayPoints(i + 1);
      upbound(2 * d_order + i) = wayPoints(i + 1);
    }

    // 更新边界
    slover.updateBounds(lowbound, upbound);

    // 求解
    slover.solve();

    Eigen::VectorXd poly_coef_1d = slover.getSolution();

    Eigen::MatrixXd poly_coef_1d_t = poly_coef_1d.transpose();
    //（px0 px1..pxm,py0,py1,..pym,pz0,pz1,..pzm）
    for (int k = 0; k < m; k++) {
      PolyCoeff.block(k, dim * p_num1d, 1, p_num1d) =
          poly_coef_1d_t.block(0, k * p_num1d, 1, p_num1d);
    }
  }

  // 每次调用之后需要清理变量
  slover.data()->clearHessianMatrix();
  slover.data()->clearLinearConstraintsMatrix();
  slover.clearSolverVariables();
  slover.clearSolver();

  return PolyCoeff;
}

// 获取hession矩阵，也就是矩阵Q
void MinimumJerkProblem::GetHession(const int n_seg, const int d_order,
                                    const Eigen::VectorXd &Time,
                                    Eigen::SparseMatrix<double> &hession) {

  int p_order = 2 * d_order - 1;
  int p_num1d = p_order + 1;
  hession.resize(n_seg * p_num1d, n_seg * p_num1d);

  hession.setZero();

  for (int k = 0; k < n_seg; ++k) {

    for (int i = d_order; i < p_num1d; ++i) {
      for (int j = d_order; j < p_num1d; ++j) {
        double value = 1.0 * Factorial(i) / Factorial(i - d_order) *
                       Factorial(j) / Factorial(j - d_order) /
                       (i + j - 2 * d_order + 1) *
                       pow(Time(k), i + j - 2 * d_order + 1);
        hession.insert(k * p_num1d + i, k * p_num1d + j) = value;
      }
    }
    // hession.block( k*p_num1d,k*p_num1d,p_num1d,p_num1d) = Q_k; 稀疏矩阵不支持
  }
}

// 在线性约束矩阵的指定位置插入系数，此函数
void MinimumJerkProblem::InsertCoff(const int row, const int col,
                                    Eigen::SparseMatrix<double> &linearMatrix,
                                    const double t, const int d_order,
                                    bool one_line, bool reverse) {

  int p_num1d = 2 * d_order;

  int flag = d_order;
  if (one_line) {
    flag = 1;
  }
  //最小化阶次，系数个数（3，6）
  Eigen::MatrixXd coff(d_order, p_num1d);
  // Am矩阵
  if (d_order == 4) {
    coff << 1.0, 1.0 * t, 1.0 * pow(t, 2), 1.0 * pow(t, 3), 1.0 * pow(t, 4),
        1.0 * pow(t, 5), 1.0 * pow(t, 6), 1.0 * pow(t, 7), 0.0, 1.0, 2.0 * t,
        3.0 * pow(t, 2), 4.0 * pow(t, 3), 5.0 * pow(t, 4), 6.0 * pow(t, 5),
        7.0 * pow(t, 6), 0.0, 0.0, 2.0, 6.0 * t, 12.0 * pow(t, 2),
        20.0 * pow(t, 3), 30.0 * pow(t, 4), 42.0 * pow(t, 5), 0.0, 0.0, 0.0,
        6.0, 24.0 * t, 60.0 * pow(t, 2), 120.0 * pow(t, 3), 210.0 * pow(t, 4);
  } else if (d_order == 3) {
    coff << // 0阶倒数
        1.0,
        1.0 * t, 1.0 * pow(t, 2), 1.0 * pow(t, 3), 1.0 * pow(t, 4),
        1.0 * pow(t, 5),
        // 1介倒数
        0.0, 1.0, 2.0 * t, 3.0 * pow(t, 2), 4.0 * pow(t, 3), 5.0 * pow(t, 4),
        // 2阶倒数
        0.0, 0.0, 2.0, 6.0 * t, 12.0 * pow(t, 2), 20.0 * pow(t, 3);
  } else {
    std::cout << "暂时只支持minisnap和minijerk";
  }

  if (reverse) {
    coff = coff * (-1.0);
  }
  //可以用block代替
  for (int i = 0; i < d_order && i < flag; ++i) {
    for (int j = 0; j < p_num1d; ++j) {
      linearMatrix.insert(row + i, col + j) = coff(i, j);
    }
  }
  // linearMatrix.block(row, col, d_order, p_num1d) = coff;
}

/*
[A0
                                 Am     //起始点约束
  B1
    B2
      B3
          ...
                            Bm-1        //中间点约束
A0 -A1
    A1 -A2
        A2 -A3
                .....
                      Am-2  Am-1    ]   //连续性约束
 */
// 获取等式约束矩阵，也就是矩阵Aeq
void MinimumJerkProblem::GetLinearConstraintsMatrix(
    const int n_seg, const int d_order, const Eigen::VectorXd &Time,
    Eigen::SparseMatrix<double> &linearMatrix) {

  int p_order = 2 * d_order - 1; //多项式次数，5
  int p_num1d = p_order + 1;     //多项式参数个数，6

  std::cout << "p_num1d:" << p_num1d << std::endl;
  std::cout << "n_seg:" << n_seg << std::endl;
  // m约束个数,n变量个数
  linearMatrix.resize(2 * d_order + (n_seg - 1) * (d_order + 1),
                      p_num1d * n_seg);

  // 起点约束(0,0)     A0(3*6)*P0(6*1)
  int row = 0;
  int col = 0;
  InsertCoff(row, col, linearMatrix, 0, d_order, false, false);
  std::cout << "row:" << row << std::endl;
  //终点约束（3,(6*m-1)） Am(3*6)*Pm(6*1)
  row += d_order;
  col = (n_seg - 1) * p_num1d;
  std::cout << "row:" << row << std::endl;
  std::cout << "col:" << col << std::endl;
  InsertCoff(row, col, linearMatrix, Time(n_seg - 1), d_order, false, false);

  // 中间节点的位置约束  Bm(1*6)*Pm(6*1)
  row += d_order;
  for (int k = 0; k < n_seg - 1; ++k) {
    InsertCoff(row + k, k * p_num1d, linearMatrix, Time(k), d_order, true,
               false); //只设置第一行
  }
  std::cout << "row:" << row << std::endl;
  // 连续性约束
  row += n_seg - 1;
  for (int k = 0; k < n_seg - 1; ++k) {
    InsertCoff(row, k * p_num1d, linearMatrix, Time(k), d_order, false, false);
    InsertCoff(row, (k + 1) * p_num1d, linearMatrix, 0, d_order, false, true);
    row += d_order;
  }
  std::cout << "row:" << row << std::endl;
}

// 计算多项式的阶乘
int MinimumJerkProblem::Factorial(int x) {
  int fac = 1;
  for (int i = x; i > 0; i--)
    fac = fac * i;
  return fac;
}

// 时间分配函数
Eigen::VectorXd
MinimumJerkProblem::TimeAllocation(const Eigen::MatrixXd &Path) {
  Eigen::VectorXd time(Path.rows() - 1);
  // 加速段时间
  double t_scope =
      vehicle_param_.max_parking_speed / vehicle_param_.max_acceleration;
  // 加速段距离(同时考虑加速和减速段）
  double distance_acc =
      1.0 / 2.0 * vehicle_param_.max_acceleration * t_scope * t_scope * 2.0;

  for (int k = 0; k < Path.rows() - 1; ++k) {
    Eigen::Vector3d delta = Path.row(k) - Path.row(k + 1);
    double d = std::sqrt(delta.dot(delta));

    if (d <= distance_acc) {
      time(k) = std::sqrt(d / vehicle_param_.max_acceleration);
    } else {
      time(k) =
          2 * t_scope + (d - distance_acc) / vehicle_param_.max_parking_speed;
    }
  }
  return time;
}

void MinimumJerkProblem::GetResult(
    const Eigen::MatrixXd poly_coeff, const Eigen::VectorXd time,
    const int d_order, const int dt,
    Eigen::VectorXd *time_result, //(x,y,phi,v,a,delta)
    Eigen::MatrixXd *state_result, Eigen::MatrixXd *control_result) {
  int num_ploy_coeff = d_order * 2;

  Eigen::MatrixXd P;   //(x,y,z)
  Eigen::MatrixXd dP;  //(dx,dy,dz)
  Eigen::MatrixXd ddP; //（ddx,ddy,ddz）
  //求的x,y,z,及其导数
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < time.rows() - 1; i++) {
      double t = time(j);
      int n = 0;
      while (t < time(j + 1)) {
        double t = time(j) + n * dt;
        double p_t = 0;
        double dp_t = 0;
        double ddp_t = 0;
        for (int k = 0; k < num_ploy_coeff; k++) {

          p_t = p_t + poly_coeff(3 * i + j + k, 0) * pow(t, k);
          if (k > 0)
            dp_t = dp_t + k * poly_coeff(3 * i + j + k, 0) * pow(t, k - 1);
          if (k > 1)
            ddp_t = ddp_t +
                    k * (k - 1) * poly_coeff(3 * i + j + k, 0) * pow(t, k - 2);
        }
        (*time_result)(n) = t;
        P(n, i) = p_t;
        dP(n, i) = dp_t;
        ddP(n, i) = ddp_t;
      }
    }
  }
  //求合成速度和航向角
}


#endif