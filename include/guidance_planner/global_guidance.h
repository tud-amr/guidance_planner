#ifndef __GLOBAL_GUIDANCE_H__
#define __GLOBAL_GUIDANCE_H__

#include <guidance_planner/prm.h>
#include <guidance_planner/graph_search.h>
#include <guidance_planner/types.h>

#include <guidance_planner/learning_guidance.h>
#include <guidance_planner/learning_types.h>

#include <dynamic_reconfigure/server.h>

namespace RosTools
{
  class DataSaver;
  class ROSMarkerPublisher;
}

namespace tk
{
  class spline;
}

#include "gsl/gsl_errno.h"

class IntegrationException : public std::exception
{
public:
  char *what()
  {
    return (char *)"GSL integration Exception";
  }
};

namespace GuidancePlanner
{
  class Config;
  class CubicSpline3D;
  struct GeometricPath;
  class GuidancePlannerConfig;

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
    void LoadObstacles(const std::vector<Obstacle> &obstacles, const std::vector<RosTools::Halfspace> &static_obstacles);
    void LoadReferencePath(double spline_start, std::unique_ptr<RosTools::CubicSpline2D<tk::spline>> &reference_path, double road_width = 4.);
    void LoadReferencePath(double spline_start, std::unique_ptr<RosTools::CubicSpline2D<tk::spline>> &reference_path, double road_width_left, double road_width_right);
    void SetGoals(const std::vector<Goal> &goals);

    /** @brief Additional configuration */
    void SetPRMSamplingFunction(SamplingFunction sampling_function) { prm_.SetPRMSamplingFunction(sampling_function); }
    void SetReferenceVelocity(double reference_velocity) { config_->reference_velocity_ = reference_velocity; }
    void SetTrackOnlyTheSelectedHomology() { config_->track_selected_homology_only_ = true; }

    /**
     * @brief Compute Guidance trajectories
     *
     * @return true If the search succeeded
     */
    bool Update();

    // --- RESULTS --- //
    struct OutputTrajectory
    {
    public:
      int topology_class;
      int color_;
      bool previously_selected_;
      bool is_new_topology_;

      StandaloneGeometricPath path; // Geometric Path
      CubicSpline3D spline;         // Spline

      OutputTrajectory(const GeometricPath &_path, const CubicSpline3D &_spline);
      OutputTrajectory(const OutputTrajectory &other);

      static OutputTrajectory &Empty(const Eigen::Vector2d &start, Config *config);
    };

    /** @brief Returns how many guidance trajectories were found */
    int NumberOfGuidanceTrajectories() const;

    /** @brief Returns the id of the path that is homotopically equivalent to path, -1 if none of them is */
    int GetIdSamePath(const GeometricPath &path);

    /**
     * @brief Get guidance trajectory
     * @param trajectory_id 0 returns the best trajectory and other trajectories are sequentially sorted on quality
     */
    OutputTrajectory &GetGuidanceTrajectory(int trajectory_id = 0);

    /**
     * @brief Get the homology cost of a geometric path with respect of the one with id spline_id
     *
     * @param output_id Index of the spline, with 0 the best spline and worse splines at higher indices
     * @return double The cost
     */
    double GetHomotopicCost(int output_id, const GeometricPath &path);
    std::vector<bool> PassesRight(int output_id, const Eigen::Vector2d &goal);

    /** @brief Get the ID of the used trajectory */
    int GetUsedTopologyClass() const { return previous_outputs_[selected_id_].topology_class; }
    OutputTrajectory &GetUsedTrajectory() { return previous_outputs_[selected_id_]; }

    /** @brief Set the ID of the used trajectory */
    void OverrideSelectedTrajectory(int output_id, bool set_none = false);

    /** @brief Checks if there were any paths found */
    bool Succeeded() { return (NumberOfGuidanceTrajectories() > 0); };

    // --- Other internal functionality --- //

    /** @brief For tracking computation times */
    double GetLastRuntime() { return benchmarkers_[0]->getLast(); };

    /** @brief Add some of the settings to the rqt_reconfigure window */
    void ReconfigureCallback(GuidancePlannerConfig &config, uint32_t level);

    /** @brief Reset this PRM. Removes previous nodes and all other transferred data. Use only when resetting the environment */
    void Reset();

    /** @brief Visualize the results of this class */
    void Visualize(bool highlight_selected = true, int only_path_nr = -1);

    /** @brief Visualize geometric path */
    void VisualizePath(RosTools::ROSLine &line, GeometricPath &path);

    /** @brief Export data for external analysis */
    void ExportData(RosTools::DataSaver &data_saver);

    Config *GetConfig() const { return config_.get(); };

    Eigen::Vector2d GetStart() const { return prm_.GetStart(); };                 /** @brief Get the start position */
    Eigen::Vector2d GetStartVelocity() const { return prm_.GetStartVelocity(); }; /** @brief Get the start velocity */

  private:
    /** @brief Identify paths that are homotopy equivalent by checking each pair */
    void KeepTopologyDistinctPaths(std::vector<GeometricPath> &paths);
    void IdentifyPreviousHomologies(std::vector<GlobalGuidance::OutputTrajectory> &outputs);

    void OrderOutputByHeuristic(std::vector<OutputTrajectory> &outputs); /** @brief Order splines if the splines are used */
    void OrderOutputByLearning(std::vector<OutputTrajectory> &outputs);  /** @brief Order splines using learning */

    double PathSelectionCost(const GeometricPath &path);

    /** Visualization functions */
    void VisualizeGeometricPaths(int path_nr = -1);
    void VisualizeTrajectories(bool highlight_selected = true, int path_nr = -1);
    void VisualizeSplinePoints();
    void VisualizeObstacles();
    void VisualizeDebug();

  private:
    ros::NodeHandle nh_;
    std::unique_ptr<Config> config_; // Owns the configuration

    boost::shared_ptr<dynamic_reconfigure::Server<GuidancePlannerConfig>> reconfigure_server_;
    boost::recursive_mutex reconfig_mutex_;

    // Classes for visualization
    std::unique_ptr<RosTools::ROSMarkerPublisher> ros_visuals_, ros_bspline_visuals_, ros_guidance_path_visuals_, ros_selected_visuals_, ros_obstacle_visuals_;
    std::unique_ptr<RosTools::ROSMarkerPublisher> ros_path_visuals_;

    PRM prm_;
    GraphSearch graph_search_;
    LearningGuidance learning_guidance_;

    // Join in a structure
    std::vector<GeometricPath> paths_;   // Found using path search
    std::vector<CubicSpline3D> splines_; // Fitted B-Splines (list because referred to in selected splines) -> not necessary anymore!

    // Outputs
    std::vector<OutputTrajectory> heuristic_outputs_, learning_outputs_;
    std::vector<OutputTrajectory> outputs_, previous_outputs_;
    int selected_id_ = -1;

    std::vector<int> sorted_indices_;
    std::unique_ptr<ColorManager> color_manager_;

    // Real-time data
    std::vector<Obstacle> obstacles_;
    std::vector<RosTools::Halfspace> static_obstacles_;

    Eigen::Vector2d start_;
    bool goals_set_ = false;
    std::vector<Goal> goals_;
    std::vector<double> spline_goal_costs_;
    double orientation_;
    Eigen::Vector2d start_velocity_;

    bool first_reconfigure_callback_;

    // Debugging variables
    bool no_message_sent_yet_;
    std::vector<std::unique_ptr<RosTools::Benchmarker>> benchmarkers_;
  };
}; // namespace Homotopy

#endif // __GLOBAL_GUIDANCE_H__
