#ifndef __PATHS_H__
#define __PATHS_H__

#include "guidance_planner/homotopy.h"

namespace GuidancePlanner
{

    /** @brief Segments help to keep a consistent ID per combination of Guard -> Connector -> Guard, while the Connector ID can vary */
    // Could also just add an extra ID to the node ID, i.e., the path association ID should be a segment association ID
    struct Segment
    {
        int id_;
    };

    /** @brief Defines the path association and how to merge it */
    struct PathAssociation
    {
        int id_;
        std::vector<int> segments_;

        PathAssociation()
        {
            id_ = -1;
        }

        PathAssociation(const std::vector<Node *> &nodes_)
        {
            for (auto &node : nodes_)
            {
                if (node->type_ == NodeType::CONNECTOR)
                    segments_.push_back(node->segment_association_id_);
            }

            id_ = -1;
        }

        /** @brief Merge this path association with another and determine the new IDs */
        /** @todo what about the segments */
        void Merge(const PathAssociation &other, int selected_id)
        {
            if (other.Assigned()) // If the other association is assigned
            {
                if (!Assigned()) // If this ID is unassigned
                {
                    id_ = other.id_; // Copy the assigned ID
                }
                else // Otherwise some paths must have merged in the environment
                {
                    std::cout << "Detected a merger of paths " << id_ << " and " << other.id_ << std::endl;
                    if (id_ == selected_id || other.id_ == selected_id) // If the selected ID is one of the paths, use that
                        id_ = selected_id;
                    else
                        id_ = std::min(id_, other.id_); // Otherwise pick the lowest
                }
            }
        }

        /** @brief Check if this path association matches another */
        bool Matches(const PathAssociation &other)
        {
            for (auto &segment : segments_)
            {
                bool found_segment = false;
                for (auto &other_segment : other.segments_)
                {
                    if (segment == other_segment)
                    {
                        found_segment = true;
                        break;
                    }
                }

                if (!found_segment)
                    return false;
            }

            return true;
        }

        /** @brief Check if this path contains the given segment */
        bool ContainsSegment(const int segment_id) const
        {
            for (auto &segment : segments_)
            {
                if (segment_id == segment)
                    return true;
            }

            return false;
        }

        bool AllSegmentsAssigned()
        {
            for (auto &segment : segments_)
            {
                if (segment == -1)
                    return false;
            }

            return true;
        }

        bool Assigned() const { return id_ != -1; };
    };

    /** @brief A collection of nodes that constitute a path */
    struct GeometricPath
    {
        PathAssociation association_;
        std::vector<Node *> nodes_; // The nodes of this path organized by k

        GeometricPath() = default;

        /** @brief Convert a vector of nodes to a path directly */
        GeometricPath(const std::vector<Node *> &nodes)
        {

            nodes_ = nodes;

            // Ensure that nodes are ordered with correct causality
            std::sort(nodes_.begin(), nodes_.end(), [](const Node *a, const Node *b)
                      { return a->point_.Time() < b->point_.Time(); });

            association_ = PathAssociation(nodes);

            ComputeDistanceVector();
        }

        /** @brief Evaluate this path at 0 <= s <= 1
         *  @return Interpolated point in discrete time space (i.e., k in [0, N]) */
        SpaceTimePoint operator()(double s) const
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

            return Helpers::InterpolateLinearly(aggregated_distances_[segment_idx], aggregated_distances_[segment_idx + 1], s, nodes_[segment_idx]->point_, nodes_[segment_idx + 1]->point_);
        }

        void SetPathID(const int id)
        {
            association_.id_ = id;
        }

        /** @brief Returns the path length in 2D*/
        double Length2D() const
        {
            double length = 0.;
            for (size_t i = 1; i < nodes_.size(); i++)
            {
                length += Helpers::dist(nodes_[i - 1]->point_.Pos(), nodes_[i]->point_.Pos()); // Length in 2D
            }

            return length;
        }

        /** @brief Returns the path length in 3D*/
        double Length3D() const
        {
            double length = 0.;
            for (size_t i = 1; i < nodes_.size(); i++)
            {
                length += (nodes_[i - 1]->point_.MapToTime() - nodes_[i]->point_.MapToTime()).norm(); /** @todo Use time scaled with dt */
            }

            return length;
        }

        /** @brief Return a smoothness measure, which divides the distance between start and end by the 3D length of the path. I.e., straighter paths are better */
        double RelativeSmoothness() const
        {
            return 1.0 - Helpers::dist(nodes_[0]->point_.Pos(), nodes_.back()->point_.Pos()) / Length3D(); // Between 0 and 1, higher costs are worse
        }

        double AverageVelocity() const
        {
            // 2D distance in a straight line, divided by time spend
            return Helpers::dist(nodes_[0]->point_.Pos(), nodes_.back()->point_.Pos()) / nodes_.back()->point_.MapToTime()(2); // Divide the length by the time
        }

        /** @brief Loop through nodes, get the lowest "k" */
        double StartTimeIndex() const
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
        double EndTimeIndex() const
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
        std::vector<Eigen::Vector3d> AsVectorOfVector3d()
        {
            std::vector<Eigen::Vector3d> result;

            for (auto &node : nodes_)
                result.emplace_back(node->point_);

            return result;
        }

        friend std::ostream &operator<<(std::ostream &stream, const GeometricPath &path)
        {
            stream << "Path: [";
            for (auto &node : path.nodes_)
                stream << *node << ", ";

            stream << "\b\b]";

            return stream;
        }

        void Clear()
        {
            nodes_.clear();
        }

    private:
        std::vector<double> aggregated_distances_; // The distance from start to each node over the path

        void ComputeDistanceVector()
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
                cur_dist += cur_node->DistanceTo(*prev_node, HomotopyConfig::DT); // Note: Time scaled
                aggregated_distances_.push_back(cur_dist);

                prev_node = cur_node;
            }
        }
    };
};

#endif // __PATHS_H__