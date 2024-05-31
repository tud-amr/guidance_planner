#include <guidance_planner/config.h>

#include <ros_tools/logging.h>

#ifndef MPC_PLANNER_ROS
#include <ros_tools/ros2_wrappers.h>
#endif

namespace GuidancePlanner
{
  // Need to be initialized outside of a member function
  bool Config::debug_output_ = false;
  bool Config::debug_visuals_ = false;
  double Config::DT = 0.05;
  double Config::CONTROL_DT = 0.0;
  int Config::N = 20;

  bool Config::use_non_passing_ = false;
  double Config::reference_velocity_ = 2.0; // Is updated based on rqt_reconfigure
  double Config::turning_radius_ = 0.5;     // Is updated based on rqt_reconfigure

  Config::Config()
  {
#ifdef MPC_PLANNER_ROS
    ros::NodeHandle node;
#else
    LOG_INFO("Get static node pointer");
    rclcpp::Node *node = GET_STATIC_NODE_POINTER();
#endif

    retrieveParameter(node, "guidance_planner/debug/output", Config::debug_output_);
    retrieveParameter(node, "guidance_planner/debug/visuals", Config::debug_visuals_);

    // High-level settings
    retrieveParameter(node, "guidance_planner/T", T_);
    retrieveParameter(node, "guidance_planner/N", Config::N);
    Config::DT = T_ / (double)Config::N;

    retrieveParameter(node, "clock_frequency", Config::CONTROL_DT, 10.); // NOTE: from LMPCC
    Config::CONTROL_DT = 1. / Config::CONTROL_DT;                        // dt = 1 / Hz

    retrieveParameter(node, "guidance_planner/seed", seed_);
    retrieveParameter(node, "guidance_planner/sampling/n_samples", n_samples_);
    retrieveParameter(node, "guidance_planner/sampling/timeout", timeout_);
    retrieveParameter(node, "guidance_planner/sampling/margin", sample_margin_, 0.);

    retrieveParameter(node, "guidance_planner/homotopy/n_paths", n_paths_);
    retrieveParameter(node, "guidance_planner/homotopy/track_selected_homology_only", track_selected_homology_only_);
    retrieveParameter(node, "guidance_planner/homotopy/comparison_function", topology_comparison_function_, std::string("Homology"));
    retrieveParameter(node, "guidance_planner/homotopy/winding/pass_threshold", winding_pass_threshold_, 0.25);
    retrieveParameter(node, "guidance_planner/homotopy/winding/use_non_passing", Config::use_non_passing_, false);
    retrieveParameter(node, "guidance_planner/homotopy/use_learning", use_learning, false);

    retrieveParameter(node, "guidance_planner/predictions_are_constant_velocity", assume_constant_velocity_);

    retrieveParameter(node, "guidance_planner/dynamics/connections", connection_type_, std::string("Straight"));
    retrieveParameter(node, "guidance_planner/dynamics/turning_radius", Config::turning_radius_, 0.5);

    retrieveParameter(node, "guidance_planner/goals/longitudinal", longitudinal_goals_);
    retrieveParameter(node, "guidance_planner/goals/vertical", vertical_goals_);

    retrieveParameter(node, "guidance_planner/max_velocity", max_velocity_);
    retrieveParameter(node, "guidance_planner/max_acceleration", max_acceleration_);

    retrieveParameter(node, "guidance_planner/connection_filters/forward", enable_forward_filter_);
    retrieveParameter(node, "guidance_planner/connection_filters/velocity", enable_velocity_filter_);
    retrieveParameter(node, "guidance_planner/connection_filters/acceleration", enable_acceleration_filter_);

    retrieveParameter(node, "guidance_planner/spline_optimization/enable", optimize_splines_);
    retrieveParameter(node, "guidance_planner/spline_optimization/geometric", geometric_weight_);
    retrieveParameter(node, "guidance_planner/spline_optimization/smoothness", smoothness_weight_);
    retrieveParameter(node, "guidance_planner/spline_optimization/collision", collision_weight_);
    retrieveParameter(node, "guidance_planner/spline_optimization/velocity_tracking", velocity_tracking_);

    // Parameters that determine the heuristic spline weighting
    retrieveParameter(node, "guidance_planner/selection_weights/length", selection_weight_length_);
    retrieveParameter(node, "guidance_planner/selection_weights/velocity", selection_weight_velocity_);
    retrieveParameter(node, "guidance_planner/selection_weights/acceleration", selection_weight_acceleration_);

    retrieveParameter(node, "guidance_planner/selection_weights/consistency", selection_weight_consistency_);

    retrieveParameter(node, "guidance_planner/spline_optimization/num_points", num_points_);
    if (num_points_ == -1)
      num_points_ = N;

    retrieveParameter(node, "guidance_planner/visuals/transparency", visuals_transparency_);
    retrieveParameter(node, "guidance_planner/visuals/visualize_all_samples", visualize_all_samples_);
    retrieveParameter(node, "guidance_planner/visuals/visualize_homology", visualize_homology_);
    retrieveParameter(node, "guidance_planner/visuals/show_indices", show_trajectory_indices_);

    retrieveParameter(node, "guidance_planner/enable/dynamically_propagate_nodes", dynamically_propagate_nodes_);
    retrieveParameter(node, "guidance_planner/enable/project_from_obstacles", project_from_obstacles_);

    retrieveParameter(node, "guidance_planner/test_node/continuous_replanning", debug_continuous_replanning_);
  }
}
