// next id: 24
class ADCTrajectory {

  //    double total_path_length; // in meters

  //    double total_path_time ; // in seconds

  //   // EStop estop = 6;

  //    //apollo.planning_internal.Debug debug = 8;

  //   // is_replan == true mean replan triggered
  //    bool is_replan ;

  //   // Specify trajectory gear
  //   // apollo.canbus.Chassis.GearPosition gear = 10;

  //   // path data + speed data
  //  // repeated apollo.common.TrajectoryPoint trajectory_point = 12;

  //   // path point without speed info
  //   //repeated apollo.common.PathPoint path_point = 13;

  //    //apollo.planning.DecisionResult decision = 14;

  //   // LatencyStats latency_stats = 15;

  //   // the routing used for current planning result
  //   // apollo.common.Header routing_header = 16;
  //   enum RightOfWayStatus { UNPROTECTED = 0; PROTECTED = 1; }
  //    RightOfWayStatus right_of_way_status = 17;

  //   // lane id along current reference line
  //   repeated apollo.hdmap.Id lane_id = 18;

  //   // set the engage advice for based on current planning result.
  //    apollo.common.EngageAdvice engage_advice = 19;

  //   // the region where planning cares most
  //   message CriticalRegion { repeated apollo.common.Polygon region = 1; }
  //   // critical region will be empty when planning is NOT sure which region
  //   is
  //   // critical
  //   // critical regions may or may not overlap
  //    CriticalRegion critical_region = 20;

  //   enum TrajectoryType {
  //     UNKNOWN = 0; NORMAL = 1; PATH_FALLBACK = 2; SPEED_FALLBACK = 3;
  //     PATH_REUSED = 4;
  //   }
  //    TrajectoryType trajectory_type = 21 [default = UNKNOWN];

  //    string replan_reason = 22;

  //   // lane id along target reference line
  //   repeated apollo.hdmap.Id target_lane_id = 23;

  //   // complete dead end flag
  //    bool car_in_dead_end = 24;

  //   // output related to RSS
  //    RSSInfo rss_info = 100;
};
