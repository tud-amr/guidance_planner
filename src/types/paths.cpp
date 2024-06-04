#include <guidance_planner/types/paths.h>

#include <guidance_planner/utils.h>

#include <ros_tools/math.h>
#include <ros_tools/spline.h>

namespace GuidancePlanner
{
    /** @brief Convert a vector of nodes to a path directly */
    GeometricPath::GeometricPath(const std::vector<Node *> &nodes)
    {
        std::vector<Node *> sorted_nodes = nodes;

        // Ensure that nodes are ordered with correct causality
        std::sort(sorted_nodes.begin(), sorted_nodes.end(), [](const Node *a, const Node *b)
                  { return a->point_.Time() < b->point_.Time(); });

        for (size_t i = 1; i < sorted_nodes.size(); i++)
        {
            // Connect the nodes
            if (Config::use_dubins_path_)
                connections_.push_back(std::make_shared<DubinsConnection>(sorted_nodes[i - 1], sorted_nodes[i]));
            else
                connections_.push_back(std::make_shared<StraightConnection>(sorted_nodes[i - 1], sorted_nodes[i]));
        }

        ComputeDistanceVector();
    }

    /** @brief Evaluate this path at 0 <= s <= 1
     *  @return Interpolated point in discrete time space (i.e., k in [0, N]) */
    SpaceTimePoint GeometricPath::operator()(double s) const
    {
        // ROSTOOLS_ASSERT(s >= 0. && s <= 1., "Geometric Path only accepts s in [0, 1]");

        if (s <= 0)
            return GetStart()->point_;

        s *= aggregated_distances_.back(); // Map to [0, D]

        for (size_t c = 0; c < connections_.size(); c++)
        {
            if (s <= aggregated_distances_[c + 1])
            {
                double local_s = s - aggregated_distances_[c];
                local_s = local_s / connections_[c]->length();
                return connections_[c]->operator()(local_s);
            }
        }

        throw std::runtime_error("GeometricPath::operator() - s out of bounds");
        return SpaceTimePoint();
    }

    bool GeometricPath::isValid(Config *config,
                                const Eigen::Vector2d &start_velocity, double start_orientation)
    {
        if (validity_checked_) // Cached validity checks
            return valid_;

        validity_checked_ = true;

        ROSTOOLS_ASSERT(connections_.size() >= 2, "Paths must have at least 2 connections");

        // Check all connections
        for (auto &connection : connections_)
        {
            if (!connection->isValid(config, start_orientation))
            {
                valid_ = false;
                return false;
            }
        }

        // Check causality
        bool causality_correct = true;
        causality_correct = causality_correct && GetStart()->type_ != NodeType::CONNECTOR;
        causality_correct = causality_correct && GetEnd()->type_ != NodeType::CONNECTOR;

        if (!causality_correct)
        {
            PRM_LOG("Path is not causal");
            valid_ = false;
            return false;
        }

        // Check accelerations
        if (config->enable_acceleration_filter_)
        {

            std::vector<double> x = {
                GetStart()->point_.Pos()(0),
                connections_[0]->getEnd()->point_.Pos()(0),
                GetEnd()->point_.Pos()(0)};
            std::vector<double> y = {
                GetStart()->point_.Pos()(1),
                connections_[0]->getEnd()->point_.Pos()(1),
                GetEnd()->point_.Pos()(1)};
            std::vector<double> t = {
                GetStart()->point_.Time() * Config::DT,
                connections_[0]->getEnd()->point_.Time() * Config::DT,
                GetEnd()->point_.Time() * Config::DT};

            // Construct a spline (with initial velocity if starting at t = 0)
            RosTools::Spline<2> spline = GetStart()->point_.Time() == 0
                                             ? RosTools::Spline<2>({x, y}, t, start_velocity)
                                             : RosTools::Spline<2>({x, y}, t);

            std::vector<double> evals = RosTools::linspace(t[0], t[2], 10); // Several points along the spline

            for (auto &t : evals)
            {
                auto acc = spline.getAcceleration(t);
                // std::cout << t << ": " << acc.norm() << std::endl;
                if (acc.norm() > config->max_acceleration_)
                {
                    PRM_LOG("Acceleration limits are not satisfied");
                    valid_ = false;
                    return false;
                }
            }
        }

        valid_ = true;
        return true;
    }

    std::vector<Node *> GeometricPath::GetNodes() const
    {
        std::vector<Node *> nodes;
        for (auto &connection : connections_)
        {
            nodes.push_back(connection->getStart());
        }
        nodes.push_back(connections_.back()->getEnd());

        return nodes;
    }

    std::vector<SpaceTimePoint> GeometricPath::GetIntegrationNodes() const
    {
        std::vector<SpaceTimePoint> integration_nodes;
        for (size_t c = 0; c < connections_.size(); c++)
        {
            connections_[c]->getIntegrationNodes(c == 0, integration_nodes);
        }
        return integration_nodes;
    }

    /** @brief Evaluate this path at a particular k **/
    std::vector<Eigen::Vector2d> GeometricPath::GetKParameterized() const
    {

        std::vector<Eigen::Vector2d> points;
        int cur_k = 1;
        points.emplace_back(GetStart()->point_.Pos());

        for (size_t i = 0; i < connections_.size(); i++)
        {
            while (cur_k < connections_[i]->getEnd()->point_.Time())
            {
                points.emplace_back(RosTools::InterpolateLinearly(
                    connections_[i]->getStart()->point_.Time(),
                    connections_[i]->getEnd()->point_.Time(),
                    cur_k,
                    connections_[i]->getStart()->point_.Pos(),
                    connections_[i]->getEnd()->point_.Pos()));

                cur_k++;
            }
        }
        points.emplace_back(GetEnd()->point_.Pos());

        return points;
    }

    double GeometricPath::Length2D() const
    {
        return aggregated_distances_.back();
    }

    double GeometricPath::Length3D() const
    {
        double length = 0.;
        for (auto &connection : connections_)
            length += connection->lengthWithTime(); // Length including the time dimension
        return length;
    }

    /** @brief Return a smoothness measure, which divides the distance between start and end by the 3D length of the path. I.e., straighter paths are
     * better */
    double GeometricPath::RelativeSmoothness() const
    {
        // Length of this path compared to the minimum path
        return Length3D() / RosTools::distance(GetStart()->point_.Pos(), GetEnd()->point_.Pos()) - 1; // Between 0 and inf, higher costs are worse
    }

    double GeometricPath::AverageVelocity() const
    {
        // 2D distance in a straight line, divided by time spend
        return RosTools::distance(GetStart()->point_.Pos(), GetEnd()->point_.Pos()) /
               GetEnd()->point_.MapToTime()(2); // Divide the length by the time
    }

    /** @brief Loop through nodes, get the lowest "k" */
    double GeometricPath::StartTimeIndex() const
    {
        return GetStart()->point_.Time();
    }

    /** @brief Loop through nodes, get the highest "k" */
    double GeometricPath::EndTimeIndex() const
    {
        return GetEnd()->point_.Time();
    }

    /** @brief Return this path as a vector of Eigen::Vector3d */
    std::vector<Eigen::Vector3d> GeometricPath::AsVectorOfVector3d()
    {
        std::vector<Eigen::Vector3d> result;

        for (auto &connection : connections_)
            result.emplace_back(connection->getStart()->point_.PosTime());

        result.emplace_back(GetEnd()->point_.PosTime());
        return result;
    }

    bool GeometricPath::ContainsNode(const Node &node) const
    {
        for (auto &connection : connections_)
        {
            if (connection->getStart()->id_ == node.id_)
                return true;
        }
        if (GetEnd()->id_ == node.id_)
            return true;

        return false;
    }

    void GeometricPath::Clear() { connections_.clear(); }

    void GeometricPath::ComputeDistanceVector()
    {
        aggregated_distances_.clear();
        aggregated_distances_.reserve(connections_.size() + 1); // Allocate space

        aggregated_distances_.push_back(0.);

        double cur_dist = 0.;
        for (auto &connection : connections_)
        {
            cur_dist += connection->length();
            aggregated_distances_.push_back(cur_dist);
        }
    }

    Node *GeometricPath::GetStart() const { return connections_[0]->getStart(); }
    Node *GeometricPath::GetEnd() const { return connections_.back()->getEnd(); }

    bool operator==(const GeometricPath &a, const GeometricPath &b)
    {

        if (a.GetConnections().size() != b.GetConnections().size())
            return false;

        for (size_t i = 0; i < a.GetConnections().size(); i++)
        {
            // If we do not have two goals (they are always equal)
            if (!(a.GetConnections()[i]->getStart()->type_ == NodeType::GOAL &&
                  b.GetConnections()[i]->getStart()->type_ == NodeType::GOAL))
            {
                if (a.GetConnections()[i]->getStart()->id_ != b.GetConnections()[i]->getStart()->id_) // Then if they do not have the same ID, these are not the same paths!
                    return false;

                if (i == a.GetConnections().size() - 1)
                {
                    if (a.GetConnections()[i]->getEnd()->id_ != b.GetConnections()[i]->getEnd()->id_) // Then if they do not have the same ID, these are not the same paths!
                        return false;
                }
            }
        }

        return true;
    }

    std::ostream &operator<<(std::ostream &stream, const GeometricPath &path)
    {
        stream << "Path: [";
        for (auto &connection : path.GetConnections())
        {
            stream << *connection->getStart() << ", ";
        }
        stream << *path.GetEnd() << ", ";
        stream << "\b\b]";

        return stream;
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
        for (auto &node : other.GetNodes())
            saved_nodes_.emplace_back(*node);

        path = GeometricPath(other.GetNodes());
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