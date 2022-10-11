#include "configs/open_space_trajectory_optimizer_config.h"

OpenSpaceTrajectoryOptimizerConfig::OpenSpaceTrajectoryOptimizerConfig(){};

OpenSpaceTrajectoryOptimizerConfig::~OpenSpaceTrajectoryOptimizerConfig(){};

const PlannerOpenSpaceConfig
OpenSpaceTrajectoryOptimizerConfig::planner_open_space_config() {

  return planner_open_space_config_;
}