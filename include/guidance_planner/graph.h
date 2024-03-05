#ifndef __HOMOTOPY_H__
#define __HOMOTOPY_H__

#include <guidance_planner/config.h>

#include <ros_tools/logging.h>
#include <ros_tools/visuals.h>

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace GuidancePlanner
{
  class Node;
  class Goal;
  /** @brief The Visibility-PRM Graph */
  class Graph
  {
  public:
    Graph(Config *config) : config_(config) {}

    Graph(const Graph &other) = delete; // No copying allowed

  public:
    Node *start_node_;
    std::vector<Node *> goal_nodes_;
    Config *config_;

    /** @brief Initialize the graph with a start and goal */
    void Initialize(const Eigen::Vector2d &start, const Eigen::Vector2d &goal);
    void Initialize(const Eigen::Vector2d &start, std::vector<Goal> &goals);

    /**
     * @brief Adds a node to the graph and returns a pointer to the node in the new list
     */
    Node *AddNode(const Node &node);

    /**
     * @brief Get a list of nodes that are neighbours of both nodes in the input list
     *
     * @param nodes A list of two nodes, that must both be guards
     * @return std::vector<Node *> A list of shared neighbours
     */
    std::vector<Node *> GetSharedNeighbours(const std::vector<Node *> &nodes);

    int GetNodeID();

    /** @brief Visualize the non-replaced vertices of this graph and there edges */
    void Visualize();
    void Clear();
    void Print();

    std::list<Node> nodes_; // @note std::list instead of std::vector to ensure that pointers remain valid

  private:
    int current_id_ = 0;
  };

} // namespace Homotopy

#endif // __HOMOTOPY_H__