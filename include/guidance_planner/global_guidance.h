#ifndef __GLOBAL_GUIDANCE_H__
#define __GLOBAL_GUIDANCE_H__

#include <chrono>
#include <unordered_map>

#include <guidance_planner/GuidancePlannerConfig.h> // Included to define the reconfigure callback

#include <ros_tools/helpers.h>
#include <ros_tools/ros_visuals.h>

#include <guidance_planner/cubic_spline.h>
#include <guidance_planner/graph_search.h>
#include <guidance_planner/config.h>
#include <guidance_planner/paths.h>
#include <guidance_planner/prm.h>

#include <third_party/spline.h>

namespace GuidancePlanner
{
class GlobalGuidance
{

public:
  GlobalGuidance();

  ~GlobalGuidance();

public:
  /**
   * @brief Set the start point for PRM
   *
   * @param start The start position
   * @param orientation The vehicle orientation at the start
   * @param velocity The initial velocity
   */
  void SetStart(const Eigen::Vector2d &start, const double orientation, const double velocity);

  /** @brief Load the obstacles to be used in the PRM, each obstacle needs to have at least the current position and N future predicted positions */
  void LoadObstacles(const std::vector<Obstacle> &obstacles, const std::vector<std::vector<Halfspace>>& static_obstacles);
  void LoadReferencePath(double spline_start, std::unique_ptr<CubicSpline2D<tk::spline>> &reference_path);
  void SetGoals(const std::vector<Goal>& goals);
  bool Update();

  /**
   * @brief Get the output guidance trajectory of this node
   *
   * @param spline_id Index of the spline, with 0 the best spline and worse splines at higher indices
   * @return CubicSpline3D& The spline object
   */
  CubicSpline3D &GetGuidanceTrajectory(int spline_id = 0);

  int GetUsedTrajectory() const;
  void SetUsedTrajectory(int spline_id);

  /** @brief Checks if there were any paths found */
  bool Succeeded() { return NumberOfGuidanceTrajectories() > 0; };

  int NumberOfGuidanceTrajectories() const;

  /** @brief For tracking computation times */
  double GetLastRuntime() { return benchmarkers_[0]->getLast(); };

  /** @brief Add some of the settings to the rqt_reconfigure window */
  void ReconfigureCallback(GuidancePlanner::GuidancePlannerConfig &config, uint32_t level);

  /** @brief Reset this PRM. Removes previous nodes and all other transferred data. Use only when resetting the environment */
  void Reset();

  /** @brief Visualize the results of this class */
  void Visualize();

  /** @brief Export data for external analysis */
  void ExportData(RosTools::DataSaver &data_saver);

  Config *GetConfig() const { return config_.get(); };

  Eigen::Vector2d GetStart() const { return prm_.GetStart(); };                 /** @brief Get the start position */
  Eigen::Vector2d GetStartVelocity() const { return prm_.GetStartVelocity(); }; /** @brief Get the start velocity */

private:
  ros::NodeHandle nh_;
  std::unique_ptr<Config> config_; // Owns the configuration

  // Classes for visualization
  std::unique_ptr<RosTools::ROSMarkerPublisher> ros_visuals_, ros_bspline_visuals_, ros_guidance_path_visuals_, ros_selected_visuals_, ros_obstacle_visuals_;
  std::unique_ptr<RosTools::ROSMarkerPublisher> ros_path_visuals_;

  PRM prm_;
  GraphSearch graph_search_;

  std::vector<GeometricPath> paths_;   // Found using path search
  std::vector<CubicSpline3D> splines_; // Fitted B-Splines (list because referred to in selected splines) -> not necessary anymore!

  // Topology propagation
  int next_segment_id_;
  std::vector<PathAssociation> known_paths_;
  std::vector<bool> path_id_was_known_;
  int selected_id_;

  // Real-time data
  std::vector<Obstacle> obstacles_;
  std::vector<std::vector<Halfspace>> static_obstacles_;

  Eigen::Vector2d start_;
  bool goals_set_ = false;
  std::vector<Goal> goals_;
  double orientation_;
  Eigen::Vector2d start_velocity_;

  bool first_reconfigure_callback_;

  // Debugging variables
  bool no_message_sent_yet_;
  std::vector<std::unique_ptr<RosTools::Benchmarker>> benchmarkers_;

  /** @brief Check for all the paths if there are any unfollowable paths and remove them if necessary */
  void FilterPaths();
  void OrderPaths();

  /** @brief Identify paths that are homotopy equivalent by checking each pair */
  void RemoveHomotopicEquivalentPaths();

  double PathSelectionCost(const GeometricPath &path);

  void AssignIDsToKnownPaths(std::vector<int> &available_ids);
  void AssignIDsToNewPaths(std::vector<int> &available_ids);

  /** Visualization functions */
  void VisualizeGeometricPaths();
  void VisualizeSplinePoints();
  void VisualizeObstacles();
  void VisualizeTrajectories();
  void VisualizeDebug();
};
}; // namespace Homotopy

#endif // __GLOBAL_GUIDANCE_H__