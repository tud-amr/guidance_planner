/**
 * @file homotopy_config.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Loads parameters for PRM
 * @version 0.1
 * @date 2022-07-12
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef HOMOTOPY_CONFIGURATION_H
#define HOMOTOPY_CONFIGURATION_H

#include <ros_tools/base_configuration.h>

#include <string>

namespace GuidancePlanner
{
  class Config : public RosTools::BaseConfiguration
  {

  public:
    Config();

    Config(const Config &other) = delete;

    /************ CONFIGURATION VARIABLES **************/

    // Debug
    static bool debug_output_;

    // Homotopy (Key variables)
    double T_;
    static int N;
    static double DT;
    static double CONTROL_DT;

    // Other statics
    static double reference_velocity_;

    // PRM Settings
    int seed_;
    double obstacle_radius_extension_;
    int n_samples_;
    double timeout_;
    bool assume_constant_velocity_;
    bool track_selected_homology_only_;
    int n_paths_;
    int path_after_samples_;
    double prefer_goal_over_smoothness_;
    double view_angle_;
    double max_velocity_, max_acceleration_;

    int longitudinal_goals_, vertical_goals_;

    bool use_learning;

    // Topology
    std::string topology_comparison_function_;
    std::string sampling_function_;
    std::string rules_;
    bool pass_left_;

    // Connection Filters
    bool enable_forward_filter_;
    bool enable_acceleration_filter_;

    // Sampling parameters
    double sample_margin_;

    // Weights (deprecated, only here so that cubicspline3d still compiles)
    double geometric_weight_, smoothness_weight_, collision_weight_, velocity_tracking_;

    bool optimize_splines_;
    double selection_weight_length_, selection_weight_velocity_, selection_weight_acceleration_;

    // Spline selection weights
    double selection_weight_consistency_;

    double visuals_transparency_;
    bool show_trajectory_indices_;

    // Spline settings
    int num_points_;

    // Toggles
    bool visualize_all_samples_, visualize_homology_;
    bool dynamically_propagate_nodes_;
    bool project_from_obstacles_;
    bool debug_continuous_replanning_;
  };
}

#endif
