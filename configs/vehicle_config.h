#pragma once

#include <string>
struct SteeringLatencyParam {
  double dead_time = 0.1;
  double rise_time = 0.17;
  double peak_time = 0.0;
  double settling_time = 0.0;
};

struct ThrottleLatencyParam {
  double dead_time = 0.03;
  double rise_time = 0.0;
  double peak_time = 0.0;
  double settling_time = 0.0;
};

struct BrakeLatencyParam {
  double dead_time = 0.0;
  double rise_time = 0.0;
  double peak_time = 0.0;
  double settling_time = 0.0;
};

class VehicleParam {

public:
  VehicleParam() = default;
  ~VehicleParam() = default;
  // string brand = "zhaokun";
  //  Car center point is car reference point, i.e., center of rear
  //  axle.后桥中心为车辆的控制中心center
  std::string brand = "LINCOLN_MKZ";
  int vehicle_id = 2;
  double front_edge_to_center = 3.89; //前面->中心的距离
  double back_edge_to_center = 1.043;
  double left_edge_to_center = 1.055;
  double right_edge_to_center = 1.055;

  double length = 4.933; //车辆长度4m
  double width = 2.11;
  double height = 1.48;

  double min_turn_radius = 5.05386147161; //最小转弯半径
  double max_acceleration = 2;
  double max_deceleration = -6;
  double max_parking_speed = 4;

  //前轮转角 = 方向盘转角 / 某个比例。
  //对应max_steer_angle_的赋值代码来看，
  // vehicle_param_.max_steer_angle()
  // 就是车辆允许的最大方向盘转角，vehicle_param_.steer_ratio()
  //就是线性关系的比例参数，而max_steer_angle_就是车辆允许的最大前轮转角。综上，steering是车辆的前轮转角大小。

  // The following items are used to compute trajectory constraints约束 in
  // planning/control/canbus,
  // vehicle max steer angle//最大方向盘转角，单位：弧度
  double max_steer_angle = 8.20304748437;
  // vehicle max steer rate; how fast can the steering wheel turn.
  double max_steer_angle_rate = 8.55211;
  // vehicle min steer rate;
  double min_steer_angle_rate = -8.55211;
  // ratio between the turn of steering wheel and the turn of wheels
  //方向盘转角与车轮转角之比 前轮转角8.2/16=29.36度
  double steer_ratio = 16;
  // the distance between the front and back wheels
  double wheel_base = 2.8448;
  // Tire effective rolling radius (vertical distance between the wheel center
  // and the ground).
  double wheel_rolling_radius = 0.335;

  // minimum differentiable vehicle speed, in m/s
  float max_abs_speed_when_stopped = 0.2;

  // minimum value get from chassis.brake, in percentage
  double brake_deadzone = 14.5;
  // minimum value get from chassis.throttle, in percentage
  double throttle_deadzone = 15.4;

  // vehicle latency parameters
  SteeringLatencyParam steering_latency_param;
  ThrottleLatencyParam throttle_latency_param;
  BrakeLatencyParam brake_latency_param;
};
