#include "guidance_planner/prm.h"

namespace GuidancePlanner
{

    PRM::~PRM()
    {
    }

    PRM::PRM()
    {
    }

    void PRM::Init(ros::NodeHandle &nh, HomotopyConfig *config)
    {
        config_ = config;

        ros_sample_visuals_.reset(new ROSMarkerPublisher(nh, "lmpcc/homotopy/all_samples", "map", 500));
        ros_graph_visuals_.reset(new ROSMarkerPublisher(nh, "lmpcc/homotopy/graph", "map", 200));
        ros_segment_visuals_.reset(new ROSMarkerPublisher(nh, "lmpcc/homotopy/segment_ids", "map", 200));

        debug_visuals_.reset(new ROSMarkerPublisher(nh, "lmpcc/homotopy/debug", "map", 200));

        graph_.reset(new Graph(config));
        environment_.Init();

        samples_.resize(config_->n_samples_);
        sample_succes_.resize(config_->n_samples_);

        done_ = false;
    }

    void PRM::LoadData(//const RealTimeData &data,
                        const std::vector<std::vector<Halfspace>>& halfspaces,
                       const std::vector<Obstacle> &obstacles,
                       const Eigen::Vector2d &start, const double orientation, const Eigen::Vector2d &velocity,
                       const std::vector<Eigen::Vector2d> &goals, const std::vector<double> &goal_costs,
                       const int previously_selected_id)
    {
        /* Obstacles */
        // obstacles_ = obstacles;
        environment_.SetPosition(start);
        if (halfspaces.size() > 0)
            environment_.LoadObstacles(obstacles, halfspaces[0]);
        else
            environment_.LoadObstacles(obstacles, std::vector<Halfspace>({}));

        // Extend the radius
        // if (config_->obstacle_radius_extension_ > 0)
        // {
        //     for (auto &obstacle : obstacles_)
        //         obstacle.radius_ += config_->obstacle_radius_extension_;
        // }

        /* Start */
        start_ = start;
        orientation_ = orientation;
        start_velocity_ = velocity; // Eigen::Vector2d(velocity * std::cos(orientation), velocity * std::sin(orientation));

        for (auto &obstacle : obstacles)
        {
            if ((obstacle.positions_[0] - start_).norm() < obstacle.radius_)
            {
                start_ = obstacle.positions_[0] + (start - obstacle.positions_[0]).normalized() * (obstacle.radius_ + 0.05);
            }
        }

        /* Goal */
        // Add goals that are collision free
        goal_costs_.clear();
        goals_.clear();
        int goal_i = 0;
        for (auto &goal : goals)
        {
            Eigen::Vector2d goal_copy = goal;                                   // Need to copy, because goal is const
            environment_.ProjectToFreeSpace(goal_copy, HomotopyConfig::N, 0.5); // Project the goal if it is in collision
            if (environment_.InCollision(SpaceTimePoint(goal_copy(0), goal_copy(1), HomotopyConfig::N)))
            {
                PRM_LOG("Rejecting a goal (it is in collision after projection).");
            }
            else
            {
                goals_.emplace_back(goal_copy);
                goal_costs_.emplace_back(goal_costs[goal_i]); // Also add its cost
            }
            goal_i++;
        }

        PRM_LOG(goals_.size() << " Collision-free goals were received");
        // }

        /* Other data */
        previously_selected_id_ = previously_selected_id;
    }

    Graph &PRM::Update()
    {
        debug_visuals_->publish(false);

        done_ = false;

        graph_->Clear();
        graph_->Initialize(start_, goals_);

        all_samples_.clear();

        next_segment_id_ = -1;

        Helpers::TriggeredTimer prm_timer(config_->timeout_ / 1000.);
        prm_timer.start();

        // Resize, because the number of samples may have been changed
        samples_.resize(config_->n_samples_);
        sample_succes_.resize(config_->n_samples_);

        // First sample all the points in parallel
        // #pragma omp parallel for num_threads(8)
        for (int i = 0; i < config_->n_samples_; i++)
        {
            sample_succes_[i] = true;
            // First resample previous nodes
            bool sample_is_from_previous_iteration = i < (int)previous_nodes_.size();

            // Get a new sample (either from the previous iteration, or a new one)
            SpaceTimePoint sample = sample_is_from_previous_iteration ? previous_nodes_[i].point_ : SampleNewPoint();

            if (config_->visualize_all_samples_)
                all_samples_.push_back(sample);

            PRM_LOG("==== [" << i << "] New Sample ====\n"
                             << sample);

            // Check if the sample is in collision
            if (environment_.InCollision(sample))
            {
                PRM_LOG("Sample was in collision. Projecting to free space");
                environment_.ProjectToFreeSpace(sample, 0.1);
                if (environment_.InCollision(sample)) // If that didn't work, then try another sample
                    sample_succes_[i] = false;
            }

            if (sample_succes_[i])
            {
                if (sample_is_from_previous_iteration)
                    previous_nodes_[i].point_ = sample; // Update the previous node's position if necessary (for construction later)

                samples_[i] = sample;
            }
        }

        // Then add them to the graph
        for (int i = 0; i < config_->n_samples_; i++)
        {
            if (!sample_succes_[i])
                continue;

            bool sample_is_from_previous_iteration = i < (int)previous_nodes_.size();
            SpaceTimePoint sample = samples_[i];

            // Find the number of visible guards from this node
            std::vector<Node *> visible_guards, visible_goals;
            FindVisibleGuards(sample, visible_guards, visible_goals);

            // If there is at most one goal, we do not have to be worried about there being more than one topology for this sample
            if (visible_goals.size() == 0 && visible_guards.size() == 0)
            {
                AddGuard(i, sample);
                continue;
            }

            // First check if we found a connector, but one that only connects to a single goal
            if (visible_goals.size() <= 1 && visible_goals.size() + visible_guards.size() == 2) // We only need to connect with one goal
            {
                for (auto &goal : visible_goals)
                    visible_guards.push_back(goal); // Add the goal to the visible guards if it exists

                AddSample(i, sample, visible_guards, sample_is_from_previous_iteration); // single threaded
            }
            else if (visible_goals.size() > 1 && visible_guards.size() == 1) // In this case we connect a guard with more than one goal
            {
                PRM_LOG("New sample connects to more than one goal");

                // There could be more than one homotopy
                Node new_node = sample_is_from_previous_iteration ? Node(graph_->GetNodeID(), previous_nodes_[i]) : Node(graph_->GetNodeID(), sample, NodeType::CONNECTOR);

                std::vector<Node *> topology_distinct_goals;
                FindTopologyDistinctGoalConnections(new_node, visible_guards, visible_goals, topology_distinct_goals);

                for (auto &goal : topology_distinct_goals) // Check its homotopy agains the other goals that
                    AddSample(i, sample, {visible_guards[0], goal}, sample_is_from_previous_iteration);
            }

            // Check for a time-out
            if (prm_timer.hasFinished())
            {
                PRM_WARN("Timeout on PRM sampling (" << config_->timeout_ << "ms)");
                break;
            }
        }

        done_ = true;
        return *graph_;
    }

    void PRM::FindTopologyDistinctGoalConnections(Node &new_node,
                                                  const std::vector<Node *> &visible_guards, const std::vector<Node *> &visible_goals,
                                                  std::vector<Node *> &topology_distinct_goals)
    {
        for (auto &goal : visible_goals)
        {
            // Construct the path for this goal
            GeometricPath new_path({visible_guards[0], &new_node, goal});

            // Check the connection
            if (!ConnectionIsValid(visible_guards[0], goal, new_node.point_))
            {
                PRM_LOG("The connection to one of the goals is not valid");
                continue;
            }

            bool is_topologically_distinct = true;

            // #pragma omp parallel for num_threads(8)
            for (auto &other_goal : topology_distinct_goals) // Check its homotopy agains the distinct paths so far
            {
                // Check if our new_path is homotopically equivalent to all topology distinct goals that we know
                GeometricPath other_path({visible_guards[0], &new_node, other_goal});

                // This speeds up comparison: Checking which path is better is very cheap, so do that first
                if (FirstPathIsBetter(other_path, new_path))
                {
                    PRM_LOG("The new path is better");

                    if (AreHomotopicEquivalent(new_path, other_path))
                    {
                        PRM_LOG("Found a homotopic equivalent path to another goal");
                        is_topologically_distinct = false;

                        // Check which one to keep
                        // if (FirstPathIsBetter(other_path, new_path))
                        // {
                        other_goal = goal; // Use this goal for the topology
                        // }

                        break;
                    }
                }
            }

            if (is_topologically_distinct)
            {
                PRM_LOG("Found a path to the goal with a distinct homotopy");

                topology_distinct_goals.push_back(goal);
            }
        }
    }

    void PRM::AddSample(int i, SpaceTimePoint &sample, const std::vector<Node *> guards, bool sample_is_from_previous_iteration)
    {
        PRM_LOG("Guards: " << *guards[0] << " and " << *guards[1]);

        // Check if the proposed connection is valid
        if (!ConnectionIsValid(guards[0], guards[1], sample))
        {
            PRM_LOG("Sampled connector is not a valid connector");
            return;
        }
        else
            PRM_LOG("Connector is valid");

        Node new_node = sample_is_from_previous_iteration ? Node(graph_->GetNodeID(), previous_nodes_[i]) : Node(graph_->GetNodeID(), sample, NodeType::CONNECTOR);
        new_node.type_ = NodeType::CONNECTOR;

        // Get other nodes that connect to these two guards
        std::vector<Node *> shared_neighbours;
        shared_neighbours = graph_->GetSharedNeighbours(guards); // Get all nodes with the same neighbours. The goal guards count as one.
        PRM_LOG("Found " << shared_neighbours.size() << " shared neighbours");

        GeometricPath new_path, other_path;
        if (shared_neighbours.size() > 0)
            new_path = GeometricPath({guards[0], &new_node, guards[1]});

        bool path_is_distinct = true;
        for (auto &neighbour : shared_neighbours)
        {
            PRM_LOG("Neighbour: " << *neighbour);
            // Temporary debugging
            if (neighbour->type_ != NodeType::CONNECTOR)
            {
                graph_->Print();
                std::cout << "Node that shows up as shared neighbour: " << *(shared_neighbours[0]) << std::endl;
            }
            LMPCC_ASSERT(neighbour->type_ == NodeType::CONNECTOR, "Shared neighbours should not be guards");
            other_path = GeometricPath({guards[0], neighbour, guards[1]}); /** @todo: Incorrect! Here we should really use the other path with the associated goal */

            // Check if paths are topologically equivalent
            if (AreHomotopicEquivalent(new_path, other_path))
            {
                PRM_LOG("Segment of the new connector is homotopically equivalent to that of " << *neighbour);
                path_is_distinct = false;

                // If they are and the new sample has a shorter path (ensure that there is a reasonable performance gain for switching)
                // if (new_path.Length3D() < other_path.Length3D() * (1.0 - config_->min_path_improvement_))
                if (FirstPathIsBetter(new_path, other_path))
                {
                    PRM_LOG("Replacing existing node " << *neighbour << "with faster node " << new_node << " (difference in length: " << other_path.Length3D() - new_path.Length3D() << " = " << other_path.Length3D() - new_path.Length3D() / other_path.Length3D() << "%)");
                    ReplaceConnector(new_node, neighbour, guards);
                }
                else
                {
                    PRM_LOG("Old path was shorter");
                }

                break; // We found an equivalent path - ignore other neighbours
            }
        }

        // If the path has no equivalent other paths
        if (path_is_distinct)
        {
            AddNewConnector(new_node, guards);
        }
        // }
    }

    void PRM::TransferPathInformationAndPropagate(const std::vector<GeometricPath> paths, const std::vector<PathAssociation> &known_paths)
    {
        previous_nodes_.clear();
        for (auto &node : graph_->nodes_)
        {
            if (node.replaced_ || node.id_ < 0) // Do not consider replaced nodes or the start/goal
                continue;

            int path_association_for_this_node = -1;
            if (node.type_ == NodeType::CONNECTOR) // For all connectors
            {
                // Find the related path and save it in the node and on the stack
                for (auto &path_association : known_paths)
                {
                    if (path_association.ContainsSegment(node.segment_association_id_))
                    {
                        node.belongs_to_path_ = path_association.id_;
                        path_association_for_this_node = path_association.id_;
                        break;
                    }
                }
            }

            // Propagate all the non-replaced nodes
            if (path_association_for_this_node == -1) // If they do not belong to a path we cannot resample
                PropagateNode(node);                  // (no path argument)
            else
            {
                for (auto &path : paths) // Otherwise find the path
                {
                    if (path.association_.id_ == path_association_for_this_node)
                    {
                        PropagateNode(node, &path); // And propagate with resampling
                        break;
                    }
                }
            }
        }
    }

    void PRM::PropagateNode(const Node &node, const GeometricPath *path)
    {
        // Copy the given node to save it (note: by value, because the graph will be reset)
        previous_nodes_.push_back(node);
        auto &propagated_node = previous_nodes_.back();

        // Then, we would like to propagate this node on its path so that it remains on the path at the same point in time (next iteration)
        // All our current nodes will be the same nodes in the next time step, but one step earlier in time
        if (config_->dynamically_propagate_nodes_)
        {
            propagated_node.point_.Time() = node.point_.Time() - (HomotopyConfig::CONTROL_DT / HomotopyConfig::DT); // Drop by however much discrete steps we are moving in one control iteration

            if (propagated_node.point_.Pos()(0) < start_(0)) /** @note Not robust */ //  Do not propagate nodes behind the robot
            {
                previous_nodes_.pop_back();
                return;
            }

            // If a node hits the floor when it belongs to a path, then we should resample to ensure that the path remains valid
            if (propagated_node.point_.Time() < 1)
            {
                if (propagated_node.type_ == NodeType::CONNECTOR && path != nullptr) // Resample connectors
                {
                    // Find the first node in the path that is not our current node
                    bool found_first_node = false;
                    int first_node_id = 1; // Skip the start
                    for (; first_node_id < (int)path->nodes_.size(); first_node_id++)
                    {
                        if (path->nodes_[first_node_id]->id_ != node.id_)
                        {
                            found_first_node = true;
                            break;
                        }
                    }
                    LMPCC_ASSERT(found_first_node, "Did not find the first next node in the path when resampling a previous node");

                    auto &next_node = path->nodes_[first_node_id];

                    // Sample a new point at its time index (0.5(T2 + T1) / (T_end - T_start)) \in [0, 1]
                    SpaceTimePoint new_sample = (*path)((0.5 * (next_node->point_.Time() + node.point_.Time())) / (path->EndTimeIndex() - path->StartTimeIndex())); // Sample halfway up to the next node

                    // Create a node to replace the old
                    // Node replacement_node(/*config_->n_samples_ + previous_nodes_.size()*/, new_sample, NodeType::CONNECTOR);
                    Node replacement_node(graph_->GetNodeID(), new_sample, NodeType::CONNECTOR);
                    replacement_node.SetSegmentAssociation(propagated_node.segment_association_id_); // This may not be homotopically equivalent to the previous, but at least this ID is free
                    replacement_node.belongs_to_path_ = propagated_node.belongs_to_path_;

                    // Remove the previous node and add the replacement
                    previous_nodes_.pop_back();
                    previous_nodes_.push_back(replacement_node);
                }
                else
                {
                    previous_nodes_.pop_back(); // Do not resample guards when they time-out
                }
            }
        }
    }

    /** @brief Because previous segments may occupy some of the IDs, we want the next free ID when called */
    int PRM::GetNextAvailableSegmentID()
    {
        next_segment_id_++;

        bool segment_id_is_free = false;
        while (!segment_id_is_free) // Check if the planned ID is already taken by any of the existing nodes
        {
            segment_id_is_free = true;
            for (auto &node : previous_nodes_)
            {
                if (node.segment_association_id_ == next_segment_id_)
                {
                    next_segment_id_++;
                    segment_id_is_free = false;
                    break;
                }
            }
        }

        return next_segment_id_;
    }

    SpaceTimePoint PRM::SampleNewPoint()
    {
        bool simple_sampling = true;
        SpaceTimePoint new_sample(0., 0., 0);
        if (simple_sampling) // Deprecated
        {

            // Uniform[0, 10][-5, 5]
            new_sample = SpaceTimePoint(random_generator_.Double() * (goals_.back()(0) - start_(0)),
                                        -3 + random_generator_.Double() * 6.,
                                        random_generator_.Int(HomotopyConfig::N - 2) + 1); // 1 - N-1
        }
        else
        {
            double start_velocity = start_velocity_.norm(); // Get the forward speed in the non-rotated frame

            // Sample a time index
            int random_k = random_generator_.Int(HomotopyConfig::N - 2) + 1; // 1 - (N-1)

            // Radial sampling based on vehicle limits
            // Goal: similar amount of samples per "k", following roughly the actuation limits of the vehicle
            // Some settings for the sampler
            double view_angle = config_->view_angle_;
            double max_velocity = config_->max_velocity_;
            double max_acceleration = config_->max_acceleration_;

            double maximum_radius = Helpers::dist(start_, goals_.back());

            // Find the maximum distance where a node can spawn for the randomly sampled k
            double min_vel = start_velocity;
            double max_vel = start_velocity;
            double max_dist = 0., min_dist = 0.;
            for (int i = 0; i < random_k; i++)
            {
                max_vel = std::min(max_vel + max_acceleration * HomotopyConfig::DT, max_velocity); // Either maximum acceleration or maximum velocity
                max_dist = std::min(max_dist + max_vel * HomotopyConfig::DT, maximum_radius);      // The distance we can travel increases with the maximum velocity

                // Account for maximum possible braking
                min_vel = std::max(min_vel - max_acceleration * HomotopyConfig::DT, 0.);
                min_dist = min_dist + min_vel * HomotopyConfig::DT;
            }

            // Sample u1 such that it spreads quadratically
            bool sample_in_quadratic = false;
            double u1, u2;
            while (!sample_in_quadratic)
            {
                u1 = random_generator_.Double();
                u2 = random_generator_.Double();
                if (u2 < u1 * u1) // Rejection sample quadratic distribution
                    sample_in_quadratic = true;
            }

            double random_r = min_dist + u1 * (max_dist - min_dist); // Sample a random radius smaller than the velocity
            double random_angle = random_generator_.Double() * view_angle - view_angle / 2.;

            // Construct the sample
            new_sample = SpaceTimePoint(random_r * std::cos(random_angle),
                                        random_r * std::sin(random_angle),
                                        random_k);

            // Check if the sample is acceptable in terms of rotational velocity
            double max_w = 1.0;
            double cur_psi = 0., cur_x = 0., cur_y = 0.;
            double cur_vel = start_velocity;
            for (int k = 0; k < random_k; k++)
            {
                cur_x += cur_vel * std::cos(cur_psi) * HomotopyConfig::DT; // Position
                cur_y += cur_vel * std::sin(cur_psi) * HomotopyConfig::DT;
                cur_psi += max_w * HomotopyConfig::DT;            // Orientation
                cur_vel -= max_acceleration * HomotopyConfig::DT; // Velocity (under maximum braking)

                cur_vel = std::max(0., cur_vel);

                if (std::abs(cur_psi) > M_PI_2) // Stop when we rotated 90 degrees
                    break;
            }

            // If our sample is outside of this rotational velocity area - recursively try again
            if ((new_sample(0) < cur_x && new_sample(1) > cur_y) || (new_sample(0) < cur_x && new_sample(1) < -cur_y))
                return SampleNewPoint(); // Recursively try again

            // Collapse the range of y values to -max_spread_y, max_spread_y to prevent the view range from becoming excessively large (disabled)
            // double max_spread_y = 5.0;
            // double new_y;
            // if (new_sample(1) > 0)
            //     new_y = new_sample(1) - std::floor(new_sample(1) / max_spread_y) * max_spread_y; // (modulo)
            // else
            //     new_y = new_sample(1) - std::ceil(new_sample(1) / max_spread_y) * max_spread_y; // (modulo)

            // new_sample.SetPos(Eigen::Vector2d(new_sample(0), new_y));

            // Translate and rotate to match the real data
        }
        Eigen::MatrixXd R = Helpers::rotationMatrixFromHeading(-orientation_);
        new_sample.SetPos(start_ + /*R * */ new_sample.Pos());

        return new_sample;
    }

    void PRM::FindVisibleGuards(SpaceTimePoint sample, std::vector<Node *> &visible_guards, std::vector<Node *> &visible_goals)
    {

        // std::unique_lock<std::mutex> lock(lock_);
        for (Node &node : graph_->nodes_)
        {
            if (node.type_ == NodeType::GUARD || node.type_ == NodeType::GOAL)
            {
                // If this guard marks a goal
                if (environment_.IsVisible(sample, node.point_))
                {
                    if (node.type_ == NodeType::GUARD)
                        visible_guards.push_back(&node);
                    else
                        visible_goals.push_back(&node);
                }
            }
        }

        PRM_LOG(visible_guards.size() << " guards and " << visible_goals.size() << " goals visible");
    }

    void PRM::ReplaceConnector(Node &new_node, Node *neighbour, const std::vector<Node *> &visible_guards)
    {

        // std::unique_lock<std::mutex> lock(lock_);

        // Add the new node to the graph (note that we are keeping the old one around, but setting its "replaced" flag to true)
        Node *new_node_ptr = graph_->AddNode(new_node);

        // Set its neighbours
        new_node_ptr->neighbours_.push_back(visible_guards[0]);
        new_node_ptr->neighbours_.push_back(visible_guards[1]);

        // If the new node has no association, load the association of the neighbour
        if (new_node_ptr->segment_association_id_ == -1)
        {
            new_node_ptr->SetSegmentAssociation(neighbour->segment_association_id_);
        }
        else if (neighbour->segment_association_id_ != -1 && new_node_ptr->segment_association_id_ != -1) // If both nodes have an association
        {
            PRM_LOG("Segments " << new_node_ptr->segment_association_id_ << " and " << neighbour->segment_association_id_ << " merged");

            // If they both have an association, then we need to pick one. We should prefer the previously selected ID
            for (auto &previous_path_association : known_paths_) // Find the previously selected path association
            {
                if (previous_path_association.id_ == previously_selected_id_)
                {
                    // Keep the neighbours one if it was selected, otherwise keep our own
                    if (previous_path_association.ContainsSegment(neighbour->segment_association_id_))
                        new_node_ptr->SetSegmentAssociation(neighbour->segment_association_id_);

                    break;
                }
            }
        }

        // replace the neighbours of the guards with the new node
        visible_guards[0]->ReplaceNeighbour(neighbour, new_node_ptr);
        visible_guards[1]->ReplaceNeighbour(neighbour, new_node_ptr);
    }

    void PRM::AddNewConnector(Node &new_node, const std::vector<Node *> &visible_guards)
    {
        // std::unique_lock<std::mutex> lock(lock_);

        Node *new_node_ptr = graph_->AddNode(new_node); // We add the new node

        // Set its neighbours
        new_node_ptr->neighbours_.push_back(visible_guards[0]);
        new_node_ptr->neighbours_.push_back(visible_guards[1]);

        // Each distinct new node is assigned a unique segment ID
        if (new_node_ptr->segment_association_id_ == -1)
        {
            new_node_ptr->segment_association_id_ = GetNextAvailableSegmentID();
            PRM_LOG("Node added with new segment association " << *new_node_ptr);
        }
        else
        {
            PRM_LOG("Previous node with segment association added " << *new_node_ptr);
        }

        // Add the new node to the neighbours of the visible guards
        visible_guards[0]->neighbours_.push_back(new_node_ptr);
        visible_guards[1]->neighbours_.push_back(new_node_ptr);
    }

    void PRM::AddGuard(int i, SpaceTimePoint &sample)
    {
        Node new_guard(i, sample, NodeType::GUARD); // Define the new node

        /* There is space here to check if this guard has some favourable properties */
        if (environment_.InCollision(sample, 0.1))
            return;
        // for (auto &obstacle : obstacles_) // Require additional clearance from guards to enforce some line-of-sight
        // {
        //     if ((obstacle.positions_[sample.Time()] - sample.Pos()).norm() < obstacle.radius_ + 0.1)
        //         return;
        // }

        PRM_LOG("Adding new guard");

        // // std::unique_lock<std::mutex> lock(lock_); // Adding and removing from the graph should lock the threads
        graph_->AddNode(new_guard); // Add the new guard
    }

    bool PRM::AreHomotopicEquivalent(const GeometricPath &a, const GeometricPath &b)
    {
        // Homotopy check with visualization
        if (config_->debug_output_)
        {
            ROSLine &line = debug_visuals_->getNewLine();
            line.setScale(0.05, 0.05);
            /** @note Space-time 3D UVD */
            // Instead of a time index, we have a path index to sample over [0-1]. Each sample is a point in 3D space-time
            Eigen::VectorXd path_indices = Eigen::VectorXd::LinSpaced(20, 0., 1.); /** @todo samples as configuration parameter */

            for (int i = 0; i < path_indices.size(); i++) /** @todo exclude start and finish */
            {

                if (config_->debug_output_ && done_) // Plot in the PRM (!done) or the filter stage (done)
                {
                    if (environment_.IsVisible(a(path_indices(i)), b(path_indices(i))))
                        line.setColor(0., 1., 0.); // Green = success
                    else
                        line.setColor(1., 0., 0.); // Red = failure

                    // Get the current points on both paths in continuous time
                    Eigen::Vector3d a_val = a(path_indices(i)).MapToTime();
                    Eigen::Vector3d b_val = b(path_indices(i)).MapToTime(); // Time index must have "k"
                    line.addLine(a_val, b_val);
                }

                if (!environment_.IsVisible(a(path_indices(i)), b(path_indices(i))))
                    return false;
            }
        }
        else // Without visualization when debug is disabled
        {

            /** @note Space-time 3D UVD */
            // Instead of a time index, we have a path index to sample over [0-1]. Each sample is a point in 3D space-time
            Eigen::VectorXd path_indices = Eigen::VectorXd::LinSpaced(20, 0., 1.); /** @todo samples as configuration parameter */

            for (int i = 0; i < path_indices.size(); i++) /** @todo exclude start and finish */
            {
                if (!environment_.IsVisible(a(path_indices(i)), b(path_indices(i))))
                    return false;
            }
        }

        return true; // If no collisions occured - paths are homotopic equivalents
    }

    /** @todo: Should be per connection, not all at once */
    bool PRM::ConnectionIsValid(const SpaceTimePoint &first_point, const SpaceTimePoint &second_point)
    {
        // This subfunction checks the connection between any two points

        // Connections must move forward in the "x" direction
        bool forward_connection = Eigen::Vector2d(1., 0.).transpose() * Helpers::rotationMatrixFromHeading(orientation_) * second_point.Pos() > Eigen::Vector2d(1., 0.).transpose() * Helpers::rotationMatrixFromHeading(orientation_) * first_point.Pos(); //.Pos()(0) > first_point.Pos()(0);
        if (!forward_connection)
        {
            PRM_LOG("Connection does not move forward in time");

            return false;
        }

        // Connections have a limited velocity
        double dist = Helpers::dist(first_point.Pos(), second_point.Pos());
        double vel = dist / ((double)std::abs(first_point.Time() - second_point.Time()) * HomotopyConfig::DT); // The average velocity of this connection
        bool vel_satisfies_limits = vel < config_->max_velocity_;

        if (!vel_satisfies_limits)
        {
            PRM_LOG("Connection does not satisfy velocity limits");

            return false;
        }

        return true; // Connection is valid
    }

    bool PRM::ConnectionIsValid(const Node *a, const Node *b, const SpaceTimePoint &new_point)
    {
        // We check a number of conditions here that must be satisfied by a valid connection from a -> new_point -> b

        // First make sure that the connection times are causal
        const Node *start_node = a->point_.Time() < b->point_.Time() ? a : b;
        const Node *end_node = a->point_.Time() < b->point_.Time() ? b : a;

        // Connections need to be in the same direction in time
        bool causality_correct = (start_node->point_.Time() < new_point.Time()) && (new_point.Time() < end_node->point_.Time());

        if (!causality_correct)
        {
            PRM_LOG("Three point connection is not causal (" << start_node->point_.Time() << ", " << new_point.Time() << ", " << end_node->point_.Time() << ")");

            return false;
        }

        // Check other conditions per connection
        if (!ConnectionIsValid(start_node->point_, new_point) || !ConnectionIsValid(new_point, end_node->point_))
            return false;

        // ACCELERATIONS
        // Fit time parameterized splines over the points and validate that the accelerations along this spline satisfy the limits
        SpaceTimePoint first_point = start_node->point_;
        SpaceTimePoint second_point = new_point;
        SpaceTimePoint third_point = end_node->point_;

        std::vector<double> t, x, y;
        t.push_back(first_point.Time() * HomotopyConfig::DT);
        t.push_back(second_point.Time() * HomotopyConfig::DT);
        t.push_back(third_point.Time() * HomotopyConfig::DT);

        x.push_back(first_point.Pos()(0));
        x.push_back(second_point.Pos()(0));
        x.push_back(third_point.Pos()(0));

        y.push_back(first_point.Pos()(1));
        y.push_back(second_point.Pos()(1));
        y.push_back(third_point.Pos()(1));

        tk::spline connect_x, connect_y;

        if (first_point.Time() == 0) // Start with initial velocity
        {
            connect_x.set_boundary(tk::spline::bd_type::first_deriv, start_velocity_(0), tk::spline::bd_type::second_deriv, 0.);
            connect_y.set_boundary(tk::spline::bd_type::first_deriv, start_velocity_(1), tk::spline::bd_type::second_deriv, 0.);
        }
        else // No acceleration on start points
        {
            connect_x.set_boundary(tk::spline::bd_type::second_deriv, 0., tk::spline::bd_type::second_deriv, 0.);
            connect_y.set_boundary(tk::spline::bd_type::second_deriv, 0., tk::spline::bd_type::second_deriv, 0.);
        }

        connect_x.set_points(t, x);
        connect_y.set_points(t, y);

        // Check accelerations
        Eigen::ArrayXd evals = Eigen::ArrayXd::LinSpaced(3, t[0], t[2]);

        for (int i = 0; i < evals.rows(); i++)
        {
            if (Eigen::Vector2d(connect_x.deriv(2, evals(i)), connect_y.deriv(2, evals(i))).norm() > config_->max_acceleration_)
            {
                PRM_LOG("Acceleration limits are not satisfied");
                return false;
            }
        }

        return true; // This connection is valid
    }

    bool PRM::FirstPathIsBetter(const GeometricPath &first_path, const GeometricPath &second_path)
    {
        // second_path.nodes_.back()->point_.Pos()(0);

        double score_1 = (1. - config_->prefer_goal_over_smoothness_) * first_path.RelativeSmoothness(); // 3D length = Prefer smooth paths
        double score_2 = (1. - config_->prefer_goal_over_smoothness_) * second_path.RelativeSmoothness();
        // std::cout << "-----------------------" << std::endl;
        // std::cout << "Smoothness 1: " << score_1 << std::endl;
        // std::cout << "Smoothness 2: " << score_2 << std::endl;

        if (first_path.nodes_.back()->type_ == NodeType::GOAL && second_path.nodes_.back()->type_ == NodeType::GOAL)
        {
            // Bit annoying: Find which goals they are to find the associated costs
            int found = 0;
            for (size_t i = 0; i < goals_.size(); i++)
            {
                if (Helpers::dist(goals_[i], first_path.nodes_.back()->point_.Pos()) < 1e-5)
                {
                    score_1 += config_->prefer_goal_over_smoothness_ * goal_costs_[i];
                    found++;
                }
                if (Helpers::dist(goals_[i], second_path.nodes_.back()->point_.Pos()) < 1e-5)
                {
                    score_2 += config_->prefer_goal_over_smoothness_ * goal_costs_[i];
                    found++;
                }

                if (found >= 2)
                    break;
            }

            // std::cout << "Total 1: " << score_1 << std::endl;
            // std::cout << "Total 2: " << score_2 << std::endl;
        }
        return score_1 < score_2 * (1.0 - config_->min_path_improvement_);
    }

    void PRM::Reset()
    {
        PRM_LOG("Reset()");

        done_ = false;

        // Forget paths
        path_id_was_known_ = std::vector<bool>(config_->n_paths_, false);
        known_paths_.clear();

        // Forget nodes
        previous_nodes_.clear();
    }

    void PRM::Visualize()
    {
        VisualizeGraph();
        VisualizeAllSamples();

        debug_visuals_->publish(true);
    }

    void PRM::VisualizeGraph()
    {
        // NODES IN THE GRAPH - COLORED BY PATH / TYPE
        ROSPointMarker &sphere = ros_graph_visuals_->getNewPointMarker("SPHERE");
        sphere.setScale(0.3, 0.3, 0.3);

        ROSLine &edge = ros_graph_visuals_->getNewLine();
        edge.setScale(0.1, 0.1);
        edge.setColor(0., 0., 0., 1.0);

        ROSTextMarker &segment_text = ros_segment_visuals_->getNewTextMarker();
        segment_text.setScale(1.0);

        int num_guards = 0;
        int num_connectors = 0;
        for (auto &node : graph_->nodes_)
        {
            LMPCC_ASSERT(node.type_ != NodeType::NONE, "Node type needs to be defined for all nodes.");
            if (node.type_ == NodeType::GUARD || node.type_ == NodeType::GOAL)
            {
                num_guards++;
                if (node.id_ < 0)
                    sphere.setColor(1., 0.0, 0.0); // Start & End coloring
                else
                    sphere.setColor(249. / 256., 142. / 256., 9. / 256., 1.); // Guard coloring

                sphere.addPointMarker(node.point_.MapToTime());
            }
            else
            {
                num_connectors++;

                // Any connector that was not replaced
                if (!node.replaced_)
                {
                    if (node.belongs_to_path_ >= 0)
                        sphere.setColorInt(node.belongs_to_path_, config_->n_paths_);
                    else
                        sphere.setColor(0.2, 0.2, 0.2, 1.0);

                    Eigen::Vector3d node_pose = node.point_.MapToTime();
                    sphere.addPointMarker(node_pose);

                    segment_text.setText(std::to_string(node.segment_association_id_));
                    segment_text.setColorInt(node.segment_association_id_, 20);
                    segment_text.addPointMarker(node_pose + Eigen::Vector3d(0., 0.5, 0.5));
                }
            }

            if (node.type_ == NodeType::GUARD || node.type_ == NodeType::GOAL) // Guards have the correct set of neighbours
            {
                for (auto &neighbour : node.neighbours_)
                {
                    if (!neighbour->replaced_)
                        edge.addLine(node.point_.MapToTime(), neighbour->point_.MapToTime());
                }
            }
        }
        ros_graph_visuals_->publish();
        ros_segment_visuals_->publish();
    }

    void PRM::VisualizeAllSamples()
    {
        // IF ENABLED, ALL PRM SAMPLES
        if (config_->visualize_all_samples_)
        {
            ROSPointMarker &samples = ros_sample_visuals_->getNewPointMarker("SPHERE");
            samples.setScale(.15, .15, .15);
            samples.setColorInt(0);

            for (auto &sample : all_samples_)
                samples.addPointMarker(sample.MapToTime());
        }

        ros_sample_visuals_->publish();
    }
}
