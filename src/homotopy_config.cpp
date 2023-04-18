#include <guidance_planner/homotopy_config.h>

namespace GuidancePlanner
{
// Need to be initialized outside of a member function
bool HomotopyConfig::debug_output_ = false;
double HomotopyConfig::DT = 0.0;
double HomotopyConfig::CONTROL_DT = 0.0;
int HomotopyConfig::N = 20;
double HomotopyConfig::reference_velocity_ = 2.0; // Is updated based on rqt_reconfigure

HomotopyConfig::HomotopyConfig()
{
    ros::NodeHandle nh;

    retrieveParameter(nh, "prm/debug_output", HomotopyConfig::debug_output_);

    // High-level settings
    retrieveParameter(nh, "prm/T", T_);
    retrieveParameter(nh, "prm/N", HomotopyConfig::N);
    HomotopyConfig::DT = T_ / (double)HomotopyConfig::N;

    retrieveParameter(nh, "clock_frequency", HomotopyConfig::CONTROL_DT); // NOTE: from LMPCC
    HomotopyConfig::CONTROL_DT = 1. / HomotopyConfig::CONTROL_DT;         // dt = 1 / Hz

    retrieveParameter(nh, "prm/obstacle_radius_extension", obstacle_radius_extension_);
    retrieveParameter(nh, "prm/n_samples", n_samples_);
    retrieveParameter(nh, "prm/n_paths", n_paths_);
    retrieveParameter(nh, "prm/path_after_samples", path_after_samples_);
    retrieveParameter(nh, "prm/timeout", timeout_);
    retrieveParameter(nh, "prm/min_path_improvement", min_path_improvement_);
    retrieveParameter(nh, "prm/prefer_goal_over_smoothness", prefer_goal_over_smoothness_);
    retrieveParameter(nh, "prm/view_angle_times_pi", view_angle_);
    view_angle_ *= M_PI;

    retrieveParameter(nh, "prm/max_velocity", max_velocity_);
    retrieveParameter(nh, "prm/max_acceleration", max_acceleration_);

    retrieveParameter(nh, "prm/weights/geometric", geometric_weight_);
    retrieveParameter(nh, "prm/weights/smoothness", smoothness_weight_);
    retrieveParameter(nh, "prm/weights/collision", collision_weight_);
    retrieveParameter(nh, "prm/weights/velocity_tracking", velocity_tracking_);
    retrieveParameter(nh, "prm/weights/repeat_times", repeat_times_);

    retrieveParameter(nh, "prm/selection_weights/length", selection_weight_length_);
    retrieveParameter(nh, "prm/selection_weights/velocity", selection_weight_velocity_);
    retrieveParameter(nh, "prm/selection_weights/longitudinal_acceleration", selection_weight_longitudinal_acceleration_);
    retrieveParameter(nh, "prm/selection_weights/lateral_acceleration", selection_weight_lateral_acceleration_);
    retrieveParameter(nh, "prm/selection_weights/consistency", selection_weight_consistency_);

    retrieveParameter(nh, "prm/spline/num_points", num_points_);

    retrieveParameter(nh, "prm/visuals/visualize_all_samples", visualize_all_samples_);
    retrieveParameter(nh, "prm/visuals/color_splines_by_cost", color_splines_by_cost_);

    retrieveParameter(nh, "prm/enable/dynamically_propagate_nodes", dynamically_propagate_nodes_);
    retrieveParameter(nh, "prm/enable/project_from_obstacles", project_from_obstacles_);
    retrieveParameter(nh, "prm/enable/smoothen_velocity", smoothen_velocity_);
    retrieveParameter(nh, "prm/enable/multithread", multithread_);

    retrieveParameter(nh, "prm/test_node/continuous_replanning", debug_continuous_replanning_);
}
};