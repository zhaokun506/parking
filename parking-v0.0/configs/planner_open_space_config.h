class PlannerOpenSpaceConfig
{
private:
    /* data */
public:
    PlannerOpenSpaceConfig(/* args */);
    ~PlannerOpenSpaceConfig();

    struct hybrid_astar
    {
        double xy_grid_resolution = 0.3;
        double phi_grid_resolution = 0.1;
        double next_node_num = 10;
        double step_size = 0.5;
        double traj_forward_penalty = 1.0;
        double traj_back_penalty = 1.0;
        double traj_gear_switch_penalty = 10.0;
        double traj_steer_penalty = 0.0;
        double traj_steer_change_penalty = 0.0;
        double grid_a_star_xy_resolution = 0.5;
        double node_radius = 0.25;
        struct s_curve_config
        {
            double acc_weight = 1.0;
            double jerk_weight = 0.0;
            double kappa_penalty_weight = 100.0;
            double ref_s_weight = 0.1;
            double ref_v_weight = 0.0;
        };
    };

    PlannerOpenSpaceConfig::PlannerOpenSpaceConfig(/* args */)
    {
    }

    PlannerOpenSpaceConfig::~PlannerOpenSpaceConfig()
    {
    }
