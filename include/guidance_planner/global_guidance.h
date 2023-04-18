#ifndef __GLOBAL_GUIDANCE_H__
#define __GLOBAL_GUIDANCE_H__

#include <chrono>
#include <unordered_map>

/** @todo Make cfg for this package */
// #include <lmpcc/PredictiveControllerConfig.h> // Included to define the reconfigure callback

#include <lmpcc_tools/helpers.h>
#include <lmpcc_tools/ros_visuals.h>

#include <guidance_planner/prm.h>
#include <guidance_planner/paths.h>
#include <guidance_planner/graph_search.h>
#include <guidance_planner/cubic_spline.h>
#include <guidance_planner/homotopy_config.h>

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
        void SetGoals(const std::vector<Eigen::Vector2d> &goal, const std::vector<double>& goal_costs); /** @brief Set the goal for PRM */

        /** @brief Load the obstacles to be used in the PRM, each obstacle needs to have at least the current position and N future predicted positions */
        void LoadObstacles(const std::vector<Obstacle> &obstacles, const std::vector<std::vector<Halfspace>>& static_obstacles);

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
        // void ReconfigureCallback(lmpcc::PredictiveControllerConfig &config, uint32_t level);

        /** @brief Reset this PRM. Removes previous nodes and all other transferred data. Use only when resetting the environment */
        void Reset();

        /** @brief Visualize the results of this class */
        void Visualize();

        /** @brief Export data for external analysis */
        void ExportData(DataSaver &data_saver);

        HomotopyConfig *GetConfig() const { return config_.get(); };

        Eigen::Vector2d GetStart() const { return prm_.GetStart(); };                 /** @brief Get the start position */
        Eigen::Vector2d GetStartVelocity() const { return prm_.GetStartVelocity(); }; /** @brief Get the start velocity */

    private:
        ros::NodeHandle nh_;
        std::unique_ptr<HomotopyConfig> config_; // Owns the configuration

        // Classes for visualization
        std::unique_ptr<ROSMarkerPublisher> ros_visuals_, ros_bspline_visuals_, ros_guidance_path_visuals_, ros_selected_visuals_, ros_obstacle_visuals_;
        std::unique_ptr<ROSMarkerPublisher> ros_path_visuals_;

        PRM prm_;
        GraphSearch graph_search_;

        Helpers::RandomGenerator random_generator_; // Used to generate samples

        std::vector<GeometricPath> paths_;   // Found using path search
        std::vector<CubicSpline3D> splines_; // Fitted B-Splines (list because referred to in selected splines) -> not necessary anymore!

        // Topology propagation
        int next_segment_id_;
        std::vector<PathAssociation> known_paths_;
        std::vector<bool> path_id_was_known_;

        // Spline selection
        // std::vector<std::shared_ptr<CubicSpline3D>> output_splines_;
        std::vector<double> spline_costs_;
        int selected_id_;
        int selected_id_iterations_;

        // Real-time data
        std::vector<Obstacle> obstacles_;
        std::vector<std::vector<Halfspace>> static_obstacles_;
        Eigen::Vector2d start_;
        std::vector<Eigen::Vector2d> goals_;
        std::vector<double> goal_costs_;
        Eigen::Vector2d previous_position_, previous_velocity_;
        double orientation_;
        Eigen::Vector2d start_velocity_;

        bool first_reconfigure_callback_;

        int max_horizon_; // Save the initial horizon before reducing it for resetting
        Eigen::Vector2d initial_goal_;

        // Debugging variables
        bool no_message_sent_yet_;
        std::vector<std::unique_ptr<Helpers::Benchmarker>> benchmarkers_;

        /** @brief Check for all the paths if there are any unfollowable paths and remove them if necessary */
        void FilterPaths();

        /** @brief Identify paths that are homotopy equivalent by checking each pair */
        void RemoveHomotopicEquivalentPaths();

        /** Visualization functions */
        void VisualizeGeometricPaths();
        void VisualizeSplinePoints();
        void VisualizeObstacles();
        void VisualizeTrajectories();
        void VisualizeDebug();
    };
};

#endif // __GLOBAL_GUIDANCE_H__