#pragma once

#include "planner_open_space_config.h"

class OpenSpaceTrajectoryOptimizerConfig {
private:
  PlannerOpenSpaceConfig planner_open_space_config_;

public:
  OpenSpaceTrajectoryOptimizerConfig(/* args */);
  ~OpenSpaceTrajectoryOptimizerConfig();
  const PlannerOpenSpaceConfig planner_open_space_config();
};
