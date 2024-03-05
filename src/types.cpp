#include <guidance_planner/types.h>

#include <ros_tools/logging.h>

namespace GuidancePlanner
{

    ColorManager::ColorManager(int size)
    {
        Reset(size);
    }

    void ColorManager::Reset(int size) { color_idxs = std::vector<bool>(size, false); }

    void ColorManager::ReleaseColor(int idx)
    {
        color_idxs[idx] = false;
    }

    bool ColorManager::ClaimColor(int idx)
    {
        if (color_idxs[idx])
            return false;

        color_idxs[idx] = true;
        return true;
    }

    int ColorManager::GetColor()
    {
        for (size_t i = 0; i < color_idxs.size(); i++)
        {
            if (!color_idxs[i])
            {
                color_idxs[i] = true;
                return i;
            }
        }

        throw std::runtime_error("No colors available (too many trajectories!)");
    }

    Obstacle::Obstacle(int id, const Eigen::Vector2d &pos, const Eigen::Vector2d &velocity, double dt, int N, double radius)
        : id_(id), radius_(radius)
    {
        Eigen::Vector2d cur_pos = pos;
        for (int k = 0; k < N + 1; k++) // Include the initial position
        {
            positions_.push_back(cur_pos);
            cur_pos += velocity * dt;
        }
    }

    Obstacle::Obstacle(int id, const std::vector<Eigen::Vector2d> &positions, double radius)
    {
        id_ = id;
        positions_ = positions;
        radius_ = radius;
    }

    Node::Node(int id, const SpaceTimePoint &point, const NodeType &node_type)
        : id_(id), point_(point)
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

    double Node::DistanceTo(const Node &other) { return (other.point_.MapToTime() - this->point_.MapToTime()).norm(); }
}