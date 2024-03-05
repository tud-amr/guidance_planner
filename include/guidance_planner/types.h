/**
 * @file types.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Defines basic classes for use in LMPCC
 * @version 0.1
 * @date 2022-05-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __GUIDANCE_TYPES_H__
#define __GUIDANCE_TYPES_H__

#include <guidance_planner/config.h>

#include <guidance_planner/utils.h>

#include <Eigen/Dense>

#include <vector>
#include <string>

namespace GuidancePlanner
{

    struct ColorManager
    {

        std::vector<bool> color_idxs;

        ColorManager(int size);

        void Reset(int size);

        void ReleaseColor(int idx);

        /**
         * @brief Try to claim a color (return false if already claimed)
         *
         * @param idx Index of color
         * @return true If color is available
         * @return false If color is claimed
         */
        bool ClaimColor(int idx);
        int GetColor();
    };

    struct Halfspace
    {
        Eigen::Vector2d A_;
        double b_;

        Halfspace(){};

        Halfspace(const Eigen::Vector2d &A, const double b) : A_(A), b_(b) {}

        static Halfspace &Dummy()
        {
            static Halfspace dummy(Eigen::Vector2d(1, 0), 1000);
            return dummy;
        }
    };

    /** Temporary */
    struct Obstacle
    {

        int id_;
        std::vector<Eigen::Vector2d> positions_;
        double radius_;

        Obstacle(int id, const Eigen::Vector2d &pos, const Eigen::Vector2d &velocity, double dt, int N, double radius);
        Obstacle(int id, const std::vector<Eigen::Vector2d> &positions, double radius);
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
        SpaceTimePoint &operator=(const SpaceTimePoint &other)
        {
            this->vec = other.vec;
            return *this;
        }

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

        // int segment_association_id_ = -1; // The identification of the path this node belongs to
        bool replaced_; // True if another node replaced this node

        int belongs_to_path_ = -1; /** @note Set a posteriori for visualization */

        std::vector<Node *> neighbours_; // Neighbouring nodes

        Node(int id, const SpaceTimePoint &point, const NodeType &node_type);
        Node(int id, const Node &other);

        /**
         * @brief Replace a neighbour of this node with a different node
         *
         * @param node_to_replace pointer to the node to replace
         * @param new_node pointer to the new node
         */
        bool ReplaceNeighbour(Node *node_to_replace, Node *new_node);
        double DistanceTo(const Node &other);

        /** @brief overload == operation to check for ID */
        friend bool operator==(const Node &lhs, const Node &rhs) { return lhs.id_ == rhs.id_; }

        // Print the type and ID
        friend std::ostream &operator<<(std::ostream &stream, const Node &node)
        {
            if (node.replaced_) // Don't print replaced nodes
                return stream;

            if (node.type_ == NodeType::CONNECTOR)
                stream << "C" << node.id_; /* << " (" << node.segment_association_id_ << ") ";*/
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

}
#endif // __GUIDANCE_TYPES_H__