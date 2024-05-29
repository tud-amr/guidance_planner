#include <guidance_planner/types/node.h>

#include <guidance_planner/config.h> // Remove later

namespace GuidancePlanner
{
    Node::Node(int id, const SpaceTimePoint &point, const NodeType &node_type) : id_(id), point_(point)
    {
        type_ = node_type;
        replaced_ = false;
    }

    Node::Node(int id, const Node &other) : id_(id), point_(other.point_)
    {
        type_ = other.type_;
        replaced_ = false;
    }

    bool Node::ReplaceNeighbour(Node *node_to_replace, Node *new_node)
    {
        for (auto &neighbour : neighbours_)
        {
            if (neighbour->id_ == node_to_replace->id_)
            {
                neighbour = new_node;
                node_to_replace->replaced_ = true;
                return true;
            }
        }

        return false;
    }

    double Node::DistanceTo(const Node &other)
    {
        return (other.point_.MapToTime() - point_.MapToTime()).norm();
    }

    bool operator==(const Node &lhs, const Node &rhs)
    {
        return lhs.id_ == rhs.id_;
    }

    std::ostream &operator<<(std::ostream &stream, const Node &node)
    {
        if (node.replaced_)
            return stream;

        if (node.type_ == NodeType::CONNECTOR)
            stream << "C" << node.id_;
        else if (node.type_ == NodeType::GOAL)
            stream << "GOAL" << node.id_;
        else
            stream << "G" << node.id_;

        return stream;
    }
} // namespace GuidancePlanner
