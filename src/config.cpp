#include <guidance_planner/config.h>

namespace GuidancePlanner
{
  // Need to be initialized outside of a member function
  bool Config::debug_output_ = false;
  double Config::DT = 0.0;
  double Config::CONTROL_DT = 0.0;
  int Config::N = 20;
  double Config::reference_velocity_ = 2.0; // Is updated based on rqt_reconfigure

  Config::Config()
  {
    ros::NodeHandle nh;

    retrieveParameter(nh, "prm/debug_output", Config::debug_output_);

    // High-level settings
    retrieveParameter(nh, "prm/T", T_);
    retrieveParameter(nh, "prm/N", Config::N);
    Config::DT = T_ / (double)Config::N;

    retrieveParameter(nh, "clock_frequency", Config::CONTROL_DT); // NOTE: from LMPCC
    Config::CONTROL_DT = 1. / Config::CONTROL_DT;                 // dt = 1 / Hz

    retrieveParameter(nh, "prm/seed", seed_);
    retrieveParameter(nh, "prm/n_samples", n_samples_);
    retrieveParameter(nh, "prm/n_paths", n_paths_);
    retrieveParameter(nh, "prm/timeout", timeout_);

    retrieveParameter(nh, "prm/predictions_are_constant_velocity", assume_constant_velocity_);
    retrieveParameter(nh, "prm/track_selected_homology_only", track_selected_homology_only_);
    retrieveParameter(nh, "prm/min_path_improvement", min_path_improvement_);

    retrieveParameter(nh, "prm/view_angle_times_pi", view_angle_);
    view_angle_ *= M_PI;

    retrieveParameter(nh, "prm/topology_comparison", topology_comparison_function_, std::string("Homology"));
    retrieveParameter(nh, "prm/use_learning", use_learning, false);
    retrieveParameter(nh, "prm/sampling_function", sampling_function_, std::string("Uniform"));
    retrieveParameter(nh, "prm/rules", rules_, std::string());
    pass_left_ = rules_ == "pass_left";

    retrieveParameter(nh, "prm/sample_margin", sample_margin_, 0.);

    retrieveParameter(nh, "prm/goals/longitudinal", longitudinal_goals_);
    retrieveParameter(nh, "prm/goals/vertical", vertical_goals_);

    retrieveParameter(nh, "prm/max_velocity", max_velocity_);
    retrieveParameter(nh, "prm/max_acceleration", max_acceleration_);

    retrieveParameter(nh, "prm/connection_filters/forward", enable_forward_filter_);
    retrieveParameter(nh, "prm/connection_filters/acceleration", enable_acceleration_filter_);

    retrieveParameter(nh, "prm/spline_optimization/enable", optimize_splines_);
    retrieveParameter(nh, "prm/spline_optimization/geometric", geometric_weight_);
    retrieveParameter(nh, "prm/spline_optimization/smoothness", smoothness_weight_);
    retrieveParameter(nh, "prm/spline_optimization/collision", collision_weight_);
    retrieveParameter(nh, "prm/spline_optimization/velocity_tracking", velocity_tracking_);

    // Parameters that determine the heuristic spline weighting
    retrieveParameter(nh, "prm/selection_weights/length", selection_weight_length_);
    retrieveParameter(nh, "prm/selection_weights/velocity", selection_weight_velocity_);
    retrieveParameter(nh, "prm/selection_weights/acceleration", selection_weight_acceleration_);

    retrieveParameter(nh, "prm/selection_weights/consistency", selection_weight_consistency_);

    retrieveParameter(nh, "prm/spline_optimization/num_points", num_points_);
    if (num_points_ == -1)
      num_points_ = N;

    retrieveParameter(nh, "prm/visuals/transparency", visuals_transparency_);
    retrieveParameter(nh, "prm/visuals/visualize_all_samples", visualize_all_samples_);
    retrieveParameter(nh, "prm/visuals/visualize_homology", visualize_homology_);
    retrieveParameter(nh, "prm/visuals/show_indices", show_trajectory_indices_);

    retrieveParameter(nh, "prm/enable/dynamically_propagate_nodes", dynamically_propagate_nodes_);
    retrieveParameter(nh, "prm/enable/project_from_obstacles", project_from_obstacles_);

    retrieveParameter(nh, "prm/test_node/continuous_replanning", debug_continuous_replanning_);
  }
};
