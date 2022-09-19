#include <memory.h>
#include "open_space_trajectory_optimizer.h"
#include "open_space_trajectory_optimizer_config.h"

int main()
{
    
   

    OpenSpaceTrajectoryOptimizerConfig opt_config;
    std::unique_ptr<OpenSpaceTrajectoryOptimizer> opt = std::make_unique<OpenSpaceTrajectoryOptimizer>(opt_config);
    opt->plan();



}