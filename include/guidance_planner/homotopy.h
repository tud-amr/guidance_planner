#ifndef __HOMOTOPY_H__
#define __HOMOTOPY_H__

#include <string>
#include <unordered_map>

#include <Eigen/Dense>
#include <exception>
#include <thread>
#include <vector>

#include <guidance_planner/config.h>

#include <ros_tools/helpers.h>
#include <ros_tools/types.h>
#include <ros_tools/ros_visuals.h>

/** Logging definitions */
#define PRM_LOGGING_ENABLED 1
#if PRM_LOGGING_ENABLED == 1
#define PRM_LOG(msg)                                                                                                                                 \
  if (Config::debug_output_)                                                                                                                 \
  {                                                                                                                                                  \
    ROS_INFO_STREAM("[PRM]: " << msg);                                                                                                               \
  }
#else
#define PRM_LOG(msg)
#endif

#define PRM_WARN(msg) ROS_WARN_STREAM("[PRM]: " << msg)

#define PRM_DEBUG_ALL 0
#if PRM_DEBUG_ALL == 1
#define IF_PRM_DEBUG(x) x
#else
#define IF_PRM_DEBUG(x)
#endif

namespace GuidancePlanner
{

/** Temporary */
struct Obstacle
{

  int id_;
  std::vector<Eigen::Vector2d> positions_;
  double radius_;

  Obstacle(int id, const Eigen::Vector2d &pos, const Eigen::Vector2d &velocity, double dt, int N, double radius) : id_(id), radius_(radius)
  {
    Eigen::Vector2d cur_pos = pos;
    for (int k = 0; k < N + 1; k++) // Include the initial position
    {
      positions_.push_back(cur_pos);
      cur_pos += velocity * dt;
    }
  }

  Obstacle(int id, const std::vector<Eigen::Vector2d> &positions, double radius)
  {
    id_ = id;
    positions_ = positions;
    radius_ = radius;
  }
};

/** @brief Defines the type of node in Visibility-PRM */
enum class NodeType
{
  NONE = 0,
  GUARD = 1,
  CONNECTOR = 2,
  GOAL = 3
};

// Wrapper for vector3d
struct SpaceTimePoint
{
  Eigen::Vector3d vec;

  SpaceTimePoint(){};

  SpaceTimePoint(const Eigen::Vector3d &other) { vec = other; }

  SpaceTimePoint(const SpaceTimePoint &other) { *this = other; }

  SpaceTimePoint(double x, double y, double k) { vec = Eigen::Vector3d(x, y, k); }

  Eigen::Vector2d Pos() const { return vec.block<2, 1>(0, 0); }
  void SetPos(const Eigen::Vector2d &val) { vec.block<2, 1>(0, 0) = val; }

  double &Time() { return vec(2); }
  double Time() const { return vec(2); }

  // Const version (scale this vectors time axis with DT), i.e., map to time in seconds
  Eigen::Vector3d MapToTime() const { return Eigen::Vector3d(vec(0), vec(1), vec(2) * Config::DT); }

  double operator()(int i) const { return vec(i); }

  SpaceTimePoint operator+(const SpaceTimePoint &other) const { return SpaceTimePoint(vec + other.vec); }

  SpaceTimePoint operator-(const SpaceTimePoint &other) const { return SpaceTimePoint(vec - other.vec); }

  operator Eigen::Vector3d() const { return vec; }

  friend std::ostream &operator<<(std::ostream &stream, const SpaceTimePoint &point)
  {
    stream << "(x = " << point.vec(0) << ", y = " << point.vec(1) << ", k = " << point.vec(2) << ") ";
    return stream;
  }

  SpaceTimePoint operator*(const double &other) const { return SpaceTimePoint(other * vec); }

  friend SpaceTimePoint operator*(const double &first, const SpaceTimePoint &other) { return SpaceTimePoint(first * other.vec); }
};

/** @brief A node of the graph constructed by PRM */
struct Node
{

  int id_;               // Unique identifier of the node
  SpaceTimePoint point_; // Location in space

  NodeType type_; // Guard / Connector

  int segment_association_id_; // The identification of the path this node belongs to
  bool replaced_;              // True if another node replaced this node

  int belongs_to_path_ = -1; /** @note Set a posteriori for visualization */

  std::vector<Node *> neighbours_; // Neighbouring nodes

  Node(int id, const SpaceTimePoint &point, const NodeType &node_type) : id_(id), point_(point)
  {
    type_ = node_type;
    replaced_ = false;
    segment_association_id_ = -1;
  }

  Node(int id, const Node &other) : id_(id), point_(other.point_)
  {
    type_ = other.type_;
    segment_association_id_ = other.segment_association_id_;

    replaced_ = false;
    // belongs_to_path_ = other.belongs_to_path_;
  }

  /**
   * @brief Replace a neighbour of this node with a different node
   *
   * @param node_to_replace pointer to the node to replace
   * @param new_node pointer to the new node
   */
  bool ReplaceNeighbour(Node *node_to_replace, Node *new_node)
  {
    for (auto &neighbour : neighbours_) // Always use references in these loops
    {
      if (neighbour->id_ == node_to_replace->id_) // To be sure, using the values not the pointers
      {
        neighbour = new_node;              // Replace the neighbour
        node_to_replace->replaced_ = true; // The neighbour was replaced
        return true;
      }
    }

    return false;
  }

  void SetSegmentAssociation(const int segment_id) { segment_association_id_ = segment_id; }

  double DistanceTo(const Node &other) { return ((Eigen::Vector3d)(other.point_ - this->point_)).norm(); }

  double DistanceTo(const Node &other, double dt) { return (other.point_.MapToTime() - this->point_.MapToTime()).norm(); }

  /** @brief overload == operation to check for ID */
  friend bool operator==(const Node &lhs, const Node &rhs) { return lhs.id_ == rhs.id_; }

  // Print the type and ID
  friend std::ostream &operator<<(std::ostream &stream, const Node &node)
  {
    if (node.replaced_) // Don't print replaced nodes
      return stream;

    if (node.type_ == NodeType::CONNECTOR)
      stream << "C" << node.id_ << " (" << node.segment_association_id_ << ") ";
    else if (node.type_ == NodeType::GOAL)
      stream << "GOAL" << node.id_;
    else
      stream << "G" << node.id_;

    return stream;
  }
};

struct Goal
{
  Node *node = nullptr;
  double cost;
  Eigen::Vector2d pos;

  Goal(const Eigen::Vector2d &_pos, const double _cost)
  {

    pos = _pos;
    cost = _cost;
  };

  static const Goal &FindGoalWithNode(const std::vector<Goal> &goals, const Node *node)
  {
    for (auto &goal : goals)
    {
      if (node->id_ == goal.node->id_)
        return goal;
    }
    throw std::runtime_error("FindGoalWithNode: Goal not found!");
  }
};

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
  void Initialize(const Eigen::Vector2d &start, const Eigen::Vector2d &goal) // To PRM?
  {
    nodes_.emplace_back(-1, SpaceTimePoint(start(0), start(1), 0), NodeType::GUARD); // Add start
    start_node_ = &nodes_.back();

    goal_nodes_.clear();
    nodes_.emplace_back(-2, SpaceTimePoint(goal(0), goal(1), Config::N), NodeType::GUARD); // Add goal
    goal_nodes_.push_back(&nodes_.back());
  }

  /** @brief Initialize the graph with a start and goal */
  void Initialize(const Eigen::Vector2d &start, std::vector<Goal> &goals) // To PRM?
  {
    nodes_.emplace_back(-1, SpaceTimePoint(start(0), start(1), 0), NodeType::GUARD); // Add start
    start_node_ = &nodes_.back();

    goal_nodes_.clear();
    for (auto &goal : goals) // Create multiple goals
    {
      nodes_.emplace_back(-(int)goal_nodes_.size() - 2, SpaceTimePoint(goal.pos(0), goal.pos(1), Config::N), NodeType::GOAL); // Add goal
      goal_nodes_.push_back(&nodes_.back());
      goal.node = &nodes_.back();
    }
  }

  /**
   * @brief Adds a node to the graph and returns a pointer to the node in the new list
   */
  Node *AddNode(const Node &node)
  {

    nodes_.emplace_back(node);
    auto *result = &(nodes_.back());

    return result;
  }

  /**
   * @brief Get a list of nodes that are neighbours of both nodes in the input list
   *
   * @param nodes A list of two nodes, that must both be guards
   * @return std::vector<Node *> A list of shared neighbours
   */
  std::vector<Node *> GetSharedNeighbours(const std::vector<Node *> &nodes)
  {
    ROSTOOLS_ASSERT(nodes.size() == 2, "Expected 2 guards, but the number of nodes are not 2"); // Function only checks neighbours shared between 2 nodes

    std::vector<Node *> shared_neighbours;

    for (auto &neighbour : nodes[0]->neighbours_) // For neighbours of the first node
    {
      for (auto &other_neighbour : nodes[1]->neighbours_) // For neighbours of the second node
      {
        if (neighbour->id_ == other_neighbour->id_ ||
            (neighbour->type_ == NodeType::GOAL &&
             other_neighbour->type_ == NodeType::GOAL)) // If neighbours are the same, add them to the shared neighbours
          shared_neighbours.push_back(neighbour);
      }
    }

    return shared_neighbours;
  }

  int GetNodeID()
  {
    int return_id = current_id_;
    current_id_++;
    return return_id;
  }

  /** @brief Visualize the non-replaced vertices of this graph and there edges */
  void Visualize(RosTools::ROSMarkerPublisher *ros_visuals)
  {
    RosTools::ROSPointMarker &sphere = ros_visuals->getNewPointMarker("SPHERE");
    sphere.setScale(0.3, 0.3, 0.3);

    RosTools::ROSLine &edge = ros_visuals->getNewLine();
    edge.setScale(0.1, 0.1);
    edge.setColor(0., 0., 0., 1.0);

    int num_guards = 0;
    int num_connectors = 0;
    for (auto &node : nodes_)
    {
      ROSTOOLS_ASSERT(node.type_ != NodeType::NONE, "Node type needs to be defined for all nodes.");
      if (node.type_ == NodeType::GUARD)
      {
        num_guards++;
        if (node.id_ < 0)
          sphere.setColor(1., 0.0, 0.0);
        else
          sphere.setColor(249. / 256., 142. / 256., 9. / 256., 1.);

        sphere.addPointMarker(node.point_.MapToTime());
      }
      else
      {
        num_connectors++;

        // Any connector that was not replaced
        if (!node.replaced_)
        {
          if (node.segment_association_id_ >= 0)
            sphere.setColorInt(node.segment_association_id_, config_->n_paths_);
          else
            sphere.setColor(0.2, 0.2, 0.2, 1.0);

          Eigen::Vector3d node_pose = node.point_.MapToTime();
          sphere.addPointMarker(node_pose);
        }
      }

      if (node.type_ == NodeType::GUARD) // Guards have the correct set of neighbours
      {
        for (auto &neighbour : node.neighbours_)
        {
          edge.addLine(node.point_.MapToTime(), neighbour->point_.MapToTime());
        }
      }
    }

    PRM_LOG("In total: " << num_guards << " Guards & " << num_connectors << " Connectors");
  }

  void Clear()
  {
    nodes_.clear();
    current_id_ = 0;
  }

  void Print()
  {
    for (auto &node : nodes_)
    {
      std::cout << "Node: " << node << std::endl;
    }
  }

  std::list<Node> nodes_; // @note std::list instead of std::vector to ensure that pointers remain valid

private:
  int current_id_ = 0;
};

}; // namespace Homotopy

#endif // __HOMOTOPY_H__