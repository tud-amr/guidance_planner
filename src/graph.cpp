

#include <guidance_planner/graph.h>

#include <guidance_planner/types/types.h>

#include <guidance_planner/utils.h>

#include <ros_tools/logging.h>
#include <ros_tools/visuals.h>

namespace GuidancePlanner
{

    void Graph::Initialize(const Eigen::Vector2d &start, const Eigen::Vector2d &goal)
    {
        nodes_.emplace_back(-1, SpaceTimePoint(start(0), start(1), 0), NodeType::GUARD); // Add start
        start_node_ = &nodes_.back();

        goal_nodes_.clear();
        nodes_.emplace_back(-2, SpaceTimePoint(goal(0), goal(1), Config::N), NodeType::GUARD); // Add goal
        goal_nodes_.push_back(&nodes_.back());
    }

    void Graph::Initialize(const Eigen::Vector2d &start, std::vector<Goal> &goals)
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

    Node *Graph::AddNode(const Node &node)
    {

        nodes_.emplace_back(node);
        auto *result = &(nodes_.back());

        return result;
    }

    std::vector<Node *> Graph::GetSharedNeighbours(const std::vector<Node *> &nodes)
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

    int Graph::GetNodeID()
    {
        int return_id = current_id_;
        current_id_++;
        return return_id;
    }

    void Graph::Visualize()
    {
        auto &ros_visuals = VISUALS.getPublisher("PRM");
        auto &sphere = ros_visuals.getNewPointMarker("SPHERE");
        sphere.setScale(0.3, 0.3, 0.3);

        auto &edge = ros_visuals.getNewLine();
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
                    // Node coloring (disabled)
                    // if (node.belongs_to_path_ >= 0)
                    //   sphere.setColorInt(node.belongs_to_path_, config_->n_paths_);
                    // else
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

    void Graph::Clear()
    {
        nodes_.clear();
        current_id_ = 0;
    }

    void Graph::Print()
    {
        for (auto &node : nodes_)
        {
            std::cout << "Node: " << node << std::endl;
        }
    }
} // namespace GuidancePlanner