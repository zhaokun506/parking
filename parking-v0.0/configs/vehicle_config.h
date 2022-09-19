class VehicleParam
{

public:
    // string brand = "zhaokun";
    //  Car center point is car reference point, i.e., center of rear axle.
    int vehicle_id = 2;
    double front_edge_to_center = 3;
    double back_edge_to_center = 4;
    double left_edge_to_center = 5;
    double right_edge_to_center = 6;

    double length = 7;
    double width = 8;
    double height = 9;

    double min_turn_radius = 10;
    double max_acceleration = 11;
    double max_deceleration = 12;

    // The following items are used to compute trajectory constraints约束 in
    // planning/control/canbus,
    // vehicle max steer angle
    double max_steer_angle = 13;
    // vehicle max steer rate; how fast can the steering wheel turn.
    double max_steer_angle_rate = 14;
    // vehicle min steer rate;
    double min_steer_angle_rate = 15;
    // ratio between the turn of steering wheel and the turn of wheels
    double steer_ratio = 16;
    // the distance between the front and back wheels
    double wheel_base = 17;
    // Tire effective rolling radius (vertical distance between the wheel center
    // and the ground).
    double wheel_rolling_radius = 18;

    // minimum differentiable vehicle speed, in m/s
    float max_abs_speed_when_stopped = 19;

    // minimum value get from chassis.brake, in percentage
    double brake_deadzone = 20;
    // minimum value get from chassis.throttle, in percentage
    double throttle_deadzone = 21;

    // vehicle latency parameters
    // LatencyParam steering_latency_param = 22;
    // LatencyParam throttle_latency_param = 23;
    // LatencyParam brake_latency_param = 24;
};
