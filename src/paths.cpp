#include <guidance_planner/paths.h>

#include <ros_tools/math.h>

namespace GuidancePlanner
{

    /** @brief Convert a vector of nodes to a path directly */
    GeometricPath::GeometricPath(const std::vector<Node *> &nodes)
    {
        nodes_ = nodes;

        // Ensure that nodes are ordered with correct causality
        std::sort(nodes_.begin(), nodes_.end(), [](const Node *a, const Node *b)
                  { return a->point_.Time() < b->point_.Time(); });

        ComputeDistanceVector();
    }

    /** @brief Evaluate this path at 0 <= s <= 1
     *  @return Interpolated point in discrete time space (i.e., k in [0, N]) */
    SpaceTimePoint GeometricPath::operator()(double s) const
    {
        // LMPCC_ASSERT(s >= 0. && s <= 1., "Geometric Path only accepts s in [0, 1]");

        s *= aggregated_distances_.back(); // Map to [0, D]

        size_t segment_idx;

        // Find the correct segment
        for (segment_idx = 0; segment_idx < aggregated_distances_.size() - 1; segment_idx++)
        {
            if (s <= aggregated_distances_[segment_idx + 1]) // if the spline parameter is smaller than the start of the next segment, use this segment
                break;
        }

        return RosTools::InterpolateLinearly(aggregated_distances_[segment_idx], aggregated_distances_[segment_idx + 1], s, nodes_[segment_idx]->point_,
                                             nodes_[segment_idx + 1]->point_);
    }

    double GeometricPath::Length2D() const
    {
        double length = 0.;
        for (size_t i = 1; i < nodes_.size(); i++)
        {
            length += RosTools::distance(nodes_[i - 1]->point_.Pos(), nodes_[i]->point_.Pos()); // Length in 2D
        }

        return length;
    }

    double GeometricPath::Length3D() const
    {
        double length = 0.;
        for (size_t i = 1; i < nodes_.size(); i++)
        {
            length += (nodes_[i - 1]->point_.MapToTime() - nodes_[i]->point_.MapToTime()).norm(); /** @todo Use time scaled with dt */
        }

        return length;
    }

    /** @brief Return a smoothness measure, which divides the distance between start and end by the 3D length of the path. I.e., straighter paths are
     * better */
    double GeometricPath::RelativeSmoothness() const
    {
        // Length of this path compared to the minimum path
        return Length3D() / RosTools::distance(nodes_[0]->point_.Pos(), nodes_.back()->point_.Pos()) - 1; // Between 0 and inf, higher costs are worse
    }

    double GeometricPath::AverageVelocity() const
    {
        // 2D distance in a straight line, divided by time spend
        return RosTools::distance(nodes_[0]->point_.Pos(), nodes_.back()->point_.Pos()) /
               nodes_.back()->point_.MapToTime()(2); // Divide the length by the time
    }

    /** @brief Loop through nodes, get the lowest "k" */
    double GeometricPath::StartTimeIndex() const
    {
        double min = 1e5;
        for (auto &node : nodes_)
        {
            if (node->point_.Time() < min)
                min = node->point_.Time();
        }

        return min;
    }

    /** @brief Loop through nodes, get the highest "k" */
    double GeometricPath::EndTimeIndex() const
    {
        double max = -1;
        for (auto &node : nodes_)
        {
            if (node->point_.Time() > max)
                max = node->point_.Time();
        }

        return max;
    }

    /** @brief Return this path as a vector of Eigen::Vector3d */
    std::vector<Eigen::Vector3d> GeometricPath::AsVectorOfVector3d()
    {
        std::vector<Eigen::Vector3d> result;

        for (auto &node : nodes_)
            result.emplace_back(node->point_);

        return result;
    }

    bool GeometricPath::ContainsNode(const Node &node) const
    {
        for (auto &path_node : nodes_)
        {
            if (path_node->id_ == node.id_)
                return true;
        }
        return false;
    }

    void GeometricPath::Clear() { nodes_.clear(); }

    void GeometricPath::ComputeDistanceVector()
    {
        aggregated_distances_.clear();
        aggregated_distances_.reserve(nodes_.size() + 1); // Allocate space

        aggregated_distances_.push_back(0.);

        Node *cur_node, *prev_node;
        prev_node = nodes_[0];

        double cur_dist = 0.;
        for (size_t i = 1; i < nodes_.size(); i++)
        {
            cur_node = nodes_[i];

            // Aggregate the distances from node to node and save them in aggregated distances
            cur_dist += cur_node->DistanceTo(*prev_node); // Note: Time scaled
            aggregated_distances_.push_back(cur_dist);

            prev_node = cur_node;
        }
    }

    StandaloneGeometricPath::StandaloneGeometricPath(const std::list<Node> &nodes)
    {
        saved_nodes_ = nodes;

        std::vector<Node *> node_ptrs;
        for (auto &node : saved_nodes_)
            node_ptrs.push_back(&node);

        path = GeometricPath(node_ptrs);
    }

    StandaloneGeometricPath::StandaloneGeometricPath(const GeometricPath &other)
    {
        saved_nodes_.clear();
        for (auto &node : other.nodes_)
            saved_nodes_.emplace_back(*node);

        path = GeometricPath(other.nodes_);
    }

    // To cast to a GeometricPath
    StandaloneGeometricPath::operator GeometricPath &()
    {
        std::vector<Node *> node_ptrs;
        for (auto &node : saved_nodes_)
            node_ptrs.push_back(&node);

        path = GeometricPath(node_ptrs);

        return path;
    }

    IDAssigner::IDAssigner(int num_objects)
    {
        IDs_.resize(num_objects * 2, true);
    }

    int IDAssigner::GetID()
    {
        for (size_t i = 0; i < IDs_.size(); i++)
        {
            if (IDs_[i])
            {
                IDs_[i] = false;
                return i;
            }
        }

        return -1;
    }

    void IDAssigner::MarkIDAsUsed(int ID)
    {

        IDs_[ID] = false;
    }
}