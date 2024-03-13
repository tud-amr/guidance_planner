#include <guidance_planner/config.h>

#include <ros_tools/logging.h>

#ifndef MPC_PLANNER_ROS
#include <ros_tools/ros2_wrappers.h>
#endif

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
#ifdef MPC_PLANNER_ROS
    ros::NodeHandle node;
#else
    rclcpp::Node *node = GET_STATIC_NODE_POINTER();
#endif

    // auto config = loadYAML("guidance_planner.yaml");

    retrieveParameter(node, "prm/debug_output", Config::debug_output_);

    // High-level settings
    retrieveParameter(node, "prm/T", T_);
    retrieveParameter(node, "prm/N", Config::N);
    Config::DT = T_ / (double)Config::N;

    retrieveParameter(node, "clock_frequency", Config::CONTROL_DT, 20.); // NOTE: from LMPCC
    Config::CONTROL_DT = 1. / Config::CONTROL_DT;                        // dt = 1 / Hz

    retrieveParameter(node, "prm/seed", seed_);
    retrieveParameter(node, "prm/n_samples", n_samples_);
    retrieveParameter(node, "prm/n_paths", n_paths_);
    retrieveParameter(node, "prm/timeout", timeout_);

    retrieveParameter(node, "prm/predictions_are_constant_velocity", assume_constant_velocity_);
    retrieveParameter(node, "prm/track_selected_homology_only", track_selected_homology_only_);

    retrieveParameter(node, "prm/view_angle_times_pi", view_angle_);
    view_angle_ *= M_PI;

    retrieveParameter(node, "prm/topology_comparison", topology_comparison_function_, std::string("Homology"));
    retrieveParameter(node, "prm/use_learning", use_learning, false);
    retrieveParameter(node, "prm/sampling_function", sampling_function_, std::string("Uniform"));
    retrieveParameter(node, "prm/rules", rules_, std::string());
    pass_left_ = rules_ == "pass_left";

    retrieveParameter(node, "prm/sample_margin", sample_margin_, 0.);

    retrieveParameter(node, "prm/goals/longitudinal", longitudinal_goals_);
    retrieveParameter(node, "prm/goals/vertical", vertical_goals_);

    retrieveParameter(node, "prm/max_velocity", max_velocity_);
    retrieveParameter(node, "prm/max_acceleration", max_acceleration_);

    retrieveParameter(node, "prm/connection_filters/forward", enable_forward_filter_);
    retrieveParameter(node, "prm/connection_filters/acceleration", enable_acceleration_filter_);

    retrieveParameter(node, "prm/spline_optimization/enable", optimize_splines_);
    retrieveParameter(node, "prm/spline_optimization/geometric", geometric_weight_);
    retrieveParameter(node, "prm/spline_optimization/smoothness", smoothness_weight_);
    retrieveParameter(node, "prm/spline_optimization/collision", collision_weight_);
    retrieveParameter(node, "prm/spline_optimization/velocity_tracking", velocity_tracking_);

    // Parameters that determine the heuristic spline weighting
    retrieveParameter(node, "prm/selection_weights/length", selection_weight_length_);
    retrieveParameter(node, "prm/selection_weights/velocity", selection_weight_velocity_);
    retrieveParameter(node, "prm/selection_weights/acceleration", selection_weight_acceleration_);

    retrieveParameter(node, "prm/selection_weights/consistency", selection_weight_consistency_);

    retrieveParameter(node, "prm/spline_optimization/num_points", num_points_);
    if (num_points_ == -1)
      num_points_ = N;

    retrieveParameter(node, "prm/visuals/transparency", visuals_transparency_);
    retrieveParameter(node, "prm/visuals/visualize_all_samples", visualize_all_samples_);
    retrieveParameter(node, "prm/visuals/visualize_homology", visualize_homology_);
    retrieveParameter(node, "prm/visuals/show_indices", show_trajectory_indices_);

    retrieveParameter(node, "prm/enable/dynamically_propagate_nodes", dynamically_propagate_nodes_);
    retrieveParameter(node, "prm/enable/project_from_obstacles", project_from_obstacles_);

    retrieveParameter(node, "prm/test_node/continuous_replanning", debug_continuous_replanning_);
  }
}
