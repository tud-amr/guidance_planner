/**
 * @file prm.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Class for constructing a topology aware graph through (x, y), t space
 * @date 2022-09-23 (documented)
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __PRM_H__
#define __PRM_H__

#include <chrono>
#include <unordered_map>

#include <lmpcc_tools/helpers.h>
#include <lmpcc_tools/ros_visuals.h>

#include <guidance_planner/environment.h>
#include <guidance_planner/paths.h>
#include <guidance_planner/graph_search.h>
#include <guidance_planner/cubic_spline.h>
#include <guidance_planner/homotopy_config.h>

#include <third_party/spline.h>

namespace GuidancePlanner
{

    class PRM
    {

    public:
        PRM();
        void Init(ros::NodeHandle &nh, HomotopyConfig *config);

        ~PRM();

    public:
        Graph &Update();

        /**
         * @brief Load input data for the PRM algorithm
         *
         * @param obstacles All obstacles with predictions over the horizon
         * @param start The start position
         * @param orientation The start orientation
         * @param velocity The start velocity
         * @param goal Goal positions organized with the furthest away goal at the end of the vector
         */
        void LoadData(const std::vector<std::vector<Halfspace>>& halfspaces,
                      const std::vector<Obstacle> &obstacles,
                      const Eigen::Vector2d &start, const double orientation, const Eigen::Vector2d &velocity,
                      const std::vector<Eigen::Vector2d> &goals, const std::vector<double> &goal_costs,
                      const int previously_selected_id);

        void TransferPathInformationAndPropagate(const std::vector<GeometricPath> paths, const std::vector<PathAssociation> &known_paths);

        Eigen::Vector2d GetStart() const
        {
            return start_;
        };                                                                    /** @brief Get the start position */
        Eigen::Vector2d GetStartVelocity() const { return start_velocity_; }; /** @brief Get the start velocity */

        /** @brief Are paths a, b homotopically equivalent? Checked with Uniform Visibility Deformation */
        bool AreHomotopicEquivalent(const GeometricPath &a, const GeometricPath &b);

        bool FirstPathIsBetter(const GeometricPath &new_path, const GeometricPath &old_path);

        /** @brief Reset this PRM. Removes previous nodes and all other transferred data. Use only when resetting the environment */
        void Reset();

        /** @brief Visualize the results of this class */
        void Visualize();

        /** @brief Export data for external analysis */
        void ExportData(DataSaver &data_saver);

    private:
        // ros::NodeHandle nh_;
        // std::unique_ptr<HomotopyConfig> config_;
        HomotopyConfig *config_;

        // Classes for visualization
        std::unique_ptr<ROSMarkerPublisher> ros_sample_visuals_, ros_graph_visuals_, ros_segment_visuals_;
        std::unique_ptr<ROSMarkerPublisher> debug_visuals_;

        // Graph related classes
        std::unique_ptr<Graph> graph_;

        bool done_;

        Helpers::RandomGenerator random_generator_; // Used to generate samples

        // Topology propagation
        int next_segment_id_;
        std::vector<PathAssociation> known_paths_;
        std::vector<Node> previous_nodes_; // Save nodes from previous iterations to enforce consistency between multiple iterations
        std::vector<bool> path_id_was_known_;

        // Real-time data
        // std::vector<Obstacle> obstacles_;
        Environment environment_;
        Eigen::Vector2d start_;
        std::vector<Eigen::Vector2d> goals_;
        std::vector<double> goal_costs_;
        Eigen::Vector2d previous_position_, previous_velocity_;
        double orientation_;
        Eigen::Vector2d start_velocity_;
        int previously_selected_id_;

        std::vector<SpaceTimePoint> samples_;
        std::vector<bool> sample_succes_;

        // Debugging variables
        std::vector<SpaceTimePoint> all_samples_; // For visualizing the sampling algorithm

        /** @brief Sample a new random point */
        SpaceTimePoint SampleNewPoint();

        void FindVisibleGuards(SpaceTimePoint sample, std::vector<Node *> &visible_guards, std::vector<Node *> &visible_goals);
        void FindTopologyDistinctGoalConnections(Node &new_node,
                                                 const std::vector<Node *> &visible_guards, const std::vector<Node *> &visible_goals,
                                                 std::vector<Node *> &topology_distinct_goals);

        /** @brief Multi-threadable handling of a new sample */
        void AddSample(int i, SpaceTimePoint &sample, const std::vector<Node *> guards, bool sample_is_from_previous_iteration);

        void AddGuard(int i, SpaceTimePoint &sample);
        void AddNewConnector(Node &new_node, const std::vector<Node *> &visible_guards);
        void ReplaceConnector(Node &new_node, Node *neighbour, const std::vector<Node *> &visible_guards);

        /** @brief For PRM, check if the two given nodes may be connected */
        bool ConnectionIsValid(const Node *a, const Node *b, const SpaceTimePoint &new_point);         // Three points
        bool ConnectionIsValid(const SpaceTimePoint &first_point, const SpaceTimePoint &second_point); // Two points

        /** @brief Propagate a node from this PRM instance "t" to the next instance "t+1". Specify a path if the node belonged to a path*/
        void PropagateNode(const Node &node, const GeometricPath *path = nullptr);

        /** @brief Get the next unused segment ID. Used to assign unique identifiers to homotopic distinct path segments */
        int GetNextAvailableSegmentID();

        /** Visualization functions */
        void VisualizeGraph();
        void VisualizeAllSamples();
    };

}
#endif // __PRM_H__