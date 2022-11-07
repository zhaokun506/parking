#pragma once

#include <string>
namespace common {

class PathPoint {
public:
  PathPoint() = default;
  const double x() const { return x_; };
  const double y() const { return y_; };
  const double z() const { return z_; };

  const double theta() const { return theta_; };
  const double kappa() const { return kappa_; };
  const double s() const { return s_; };
  const double dkappa() const { return dkappa_; };
  const double ddkappa() const { return ddkappa_; };
  const double x_derivative() const { return x_derivative_; };
  const double y_derivative() const { return y_derivative_; };

  void set_x(const double x) { x_ = x; };
  void set_y(const double y) { y_ = y; };
  void set_z(const double z) { z_ = z; };
  void set_theta(const double theta) { theta_ = theta; };
  void set_kappa(const double kappa) { kappa_ = kappa; };
  void set_dkappa(const double dkappa) { dkappa_ = dkappa; };
  void set_ddkappa(const double ddkappa) { ddkappa_ = ddkappa; };
  void set_s(const double s) { s_ = s; };
  void set_x_derivative(const double x_derivative) {
    x_derivative_ = x_derivative;
  };
  void set_y_derivative(const double y_derivative) {
    y_derivative_ = y_derivative;
  };

private:
  // coordinates
  double x_;
  double y_;
  double z_;

  // direction on the x-y plane
  double theta_;
  // curvature on the x-y planning
  double kappa_;
  // accumulated distance from beginning of the path
  double s_;

  // derivative of kappa w.r.t s.
  double dkappa_;
  // derivative of derivative of kappa w.r.t s.
  double ddkappa_;
  // The lane ID where the path point is on
  std::string lane_id;

  // derivative of x and y w.r.t parametric parameter t in
  // CosThetareferenceline
  double x_derivative_;
  double y_derivative_;
};

/*------------------------------------------------------------------------------------------------------------------*/
//apollo不是继承而是包含该类成员
class TrajectoryPoint : public PathPoint {
  // path point
  // PathPoint path_point;
  // linear velocity

public:
  const PathPoint path_point() const { return path_point_; };

  PathPoint *mutable_path_point() { return &path_point_; }; //返回指针

  const double v() const { return v_; };
  const double a() const { return a_; };
  const double relative_time() const { return relative_time_; };
  const double da() const { return da_; };
  const double steer() const { return steer_; };

  void set_v(const double v) { v_ = v; };
  void set_a(const double a) { a_ = a; };
  void set_relative_time(const double relative_time) {
    relative_time_ = relative_time;
  };
  void set_da(const double da) { da_ = da; };
  void set_steer(const double steer) { steer_ = steer; };

private:
  double v_; // in [m/s]
             // linear acceleration
  double a_;
  // relative time from beginning of the trajectory
  double relative_time_;
  // longitudinal jerk
  double da_;
  // The angle between vehicle front wheel and vehicle longitudinal axis
  double steer_;

  PathPoint path_point_;
};

/*------------------------------------------------------------------------------------------------------------------*/
class SpeedPoint {
public:
  void Clear() {
    s_ = 0;
    t_ = 0;
    v_ = 0;
    a_ = 0;
    da_ = 0;
  };

  const double s() const { return s_; };
  void set_s(double s) { s_ = s; };
  bool has_s() { return (s_ != 0); };

  const double t() const { return t_; };
  void set_t(double t) { t_ = t; };

  const double v() const { return v_; };
  void set_v(double v) { v_ = v; };
  bool has_v() { return true; };

  const double a() const { return a_; };
  void set_a(double a) { a_ = a; };
  bool hav_a() { return true; };

  const double da() const { return da_; };
  void set_da(double da) { da_ = da; };
  bool hav_da() { return true; };

private:
  double s_;
  double t_;
  // speed (m/s)
  double v_;
  // acceleration (m/s^2)
  double a_;
  // jerk (m/s^3)
  double da_;
};

/*------------------------------------------------------------------------------------------------------------------*/
class SLPoint {
public:
  const double s() const { return s_; };
  const double l() const { return l_; };
  void set_s(const double s) { s_ = s; };
  void set_l(const double l) { l_ = l; };

private:
  double s_;
  double l_;
};

class FrenetFramePoint {
  double s = 1;
  double l = 2;
  double dl = 3;
  double ddl = 4;
};

} // namespace common