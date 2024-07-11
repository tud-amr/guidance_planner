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

#include <guidance_planner/cubic_spline.h>
#include <guidance_planner/environment.h>
#include <guidance_planner/graph_search.h>
#include <guidance_planner/config.h>

#include <guidance_planner/paths.h>
#include <guidance_planner/types.h>

#include <ros_tools/random_generator.h>

namespace RosTools
{
  class DataSaver;
}

namespace GuidancePlanner
{
  class PRM;
  typedef SpaceTimePoint (PRM::*SamplingFunction)();
  class TopologyComparison;

  class PRM
  {

  public:
    PRM();
    void Init(Config *config);

    ~PRM();

  public:
    /**
     * @brief Main Update
     *
     * @return Graph&
     */
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
    void LoadData(const std::vector<Obstacle> &obstacles, const std::vector<Halfspace> &static_obstacles, const Eigen::Vector2d &start, const double orientation,
                  const Eigen::Vector2d &velocity, const std::vector<Goal> &goals);

    void LoadReferencePath(std::shared_ptr<RosTools::Spline2D> reference_path, const double cur_s,
                           const double max_s, const double width);

    void SetPRMSamplingFunction(SamplingFunction sampling_function) { sampling_function_ = sampling_function; }

    void DoNotPropagateNodes() { do_not_propagate_nodes_ = true; };

    /** @brief Propagate the graph to the next iteration by lowering the time axis */
    void PropagateGraph(const std::vector<GeometricPath> &paths);

    Eigen::Vector2d GetStart() const { return start_; };
    Eigen::Vector2d GetStartVelocity() const { return start_velocity_; };
    std::vector<Goal> *GetGoals() { return &goals_; }

    /** @brief Are paths a, b in an equivalent topology class */
    bool AreHomotopicEquivalent(const GeometricPath &a, const GeometricPath &b);

    /** @brief Do we prefer the first path or the second path */
    bool FirstPathIsBetter(const GeometricPath &new_path, const GeometricPath &old_path);

    /** @brief For each obstacle compute if the path is passing left or right */
    std::vector<bool> GetLeftPassingVector(const GeometricPath &path);

    /** @brief Get homotopic cost between paths if using homologies*/
    double GetHomotopicCost(const GeometricPath &a, const GeometricPath &b);
    std::vector<bool> PassesRight(const GeometricPath &path);

    /** @brief Reset this PRM. Removes previous nodes and all other transferred data. Use only when resetting the environment */
    void Reset();

    /** @brief Visualize the results of this class */
    void Visualize();

    /** @brief Export data for external analysis */
    void saveData(RosTools::DataSaver &data_saver);

  private:
    void SampleNewPoints(std::vector<SpaceTimePoint> &samples, std::vector<bool> &sample_succes); // Sample ALL new points

    /** @brief Sample a new random point */
    SpaceTimePoint SampleNewPoint();
    SpaceTimePoint SampleUniformly3D();
    SpaceTimePoint SampleUniformly3DReferencePath();
    void FindVisibleGuards(SpaceTimePoint sample, std::vector<Node *> &visible_guards, std::vector<Node *> &visible_goals);
    Node *FindTopologyDistinctGoalConnection(Node &new_node, const std::vector<Node *> &visible_guards, std::vector<Node *> &visible_goals);

    void AddSample(int i, SpaceTimePoint &sample, const std::vector<Node *> guards, bool sample_is_from_previous_iteration);
    void AddGuard(int i, SpaceTimePoint &sample);
    void AddNewConnector(Node &new_node, const std::vector<Node *> &visible_guards);
    void ReplaceConnector(Node &new_node, Node *neighbour, const std::vector<Node *> &visible_guards);

    /** @brief Check if the two given nodes may be connected */
    bool ConnectionIsValid(const Node *a, const Node *b, const SpaceTimePoint &new_point);         // Three points
    bool ConnectionIsValid(const GeometricPath &path);                                             // Three points
    bool ConnectionIsValid(const SpaceTimePoint &first_point, const SpaceTimePoint &second_point); // Two points

    /** @brief Propagate a node from this PRM instance "t" to the next instance "t+1". Specify a path if the node belonged to a path*/
    void PropagateNode(const Node &node, const GeometricPath *path = nullptr);

    /** @brief Get the next unused segment ID. Used to assign unique identifiers to homotopic distinct path segments */
    int GetNextAvailableSegmentID();

    /** Visualization functions */
    void VisualizeGraph();
    void VisualizeAllSamples();

  private:
    bool done_;
    bool do_not_propagate_nodes_{false};

    Config *config_;

    std::unique_ptr<Graph> graph_;                            // PRM Graph
    std::unique_ptr<TopologyComparison> topology_comparison_; // H-invariant or UVD comparison

    std::shared_ptr<RosTools::Spline2D> reference_path_;

    SamplingFunction sampling_function_; /** @note Samples are relative to start position and orientation */

    RosTools::RandomGenerator random_generator_; // Used to generate samples

    std::vector<Node> previous_nodes_; // Save nodes from previous iterations to enforce consistency between multiple iterations

    // Real-time data
    Environment environment_;
    Eigen::Vector2d start_, start_velocity_;
    double orientation_;

    std::vector<Goal> goals_;
    double min_x_ = 0, min_y_ = 0, range_x_ = 1, range_y_ = 1;
    double min_s_ = 0, min_lat_ = 0, range_s_ = 1, range_lat_ = 1;

    std::vector<SpaceTimePoint> samples_;
    std::vector<bool> sample_succes_;

    // Debugging variables
    std::vector<SpaceTimePoint> all_samples_; // For visualizing the sampling algorithm
    // std::unique_ptr<RosTools::Benchmarker> debug_benchmarker_;
  };

} // namespace Homotopy
#endif // __PRM_H__