#ifndef GUIDANCE_PLANNER_NODE_H
#define GUIDANCE_PLANNER_NODE_H

#include <guidance_planner/types/space_time_point.h>
#include <guidance_planner/types/type_define.h>

#include <vector>

namespace GuidancePlanner
{
    struct Node;

    /** @brief Defines the type of NodeDimin Visibility-PRM */
    enum class NodeType
    {
        NONE = 0,
        GUARD = 1,
        CONNECTOR = 2,
        GOAL = 3
    };

    /** @brief A Node the graph constructed by PRM */
    struct Node
    {
        int id_;               // Unique identifier of the node
        SpaceTimePoint point_; // Location in space

        NodeType type_; // Guard / Connector

        bool replaced_; // True if another NodeDim replaced this node

        int belongs_to_path_ = -1; /** @note Set a posteriori for visualization */

        std::vector<Node *> neighbours_; // Neighbouring nodes

        Node(int id, const SpaceTimePoint &point, const NodeType &node_type);

        Node(int id, const Node &other);

        bool ReplaceNeighbour(Node *node_to_replace, Node *new_node);

        double DistanceTo(const Node &other);

        friend bool operator==(const Node &lhs, const Node &rhs);

        friend std::ostream &operator<<(std::ostream &stream, const Node &node);
    };
} // namespace GuidancePlanner

#endif // GUIDANCE_PLANNER_NODE_H
