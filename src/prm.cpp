#include "guidance_planner/prm.h"

namespace GuidancePlanner
{

  PRM::~PRM() {}

  PRM::PRM() {}

  void PRM::Init(ros::NodeHandle &nh, Config *config)
  {
    config_ = config;

    ros_sample_visuals_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/homotopy/all_samples", "map", 500));
    ros_graph_visuals_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/homotopy/graph", "map", 200));
    ros_segment_visuals_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/homotopy/segment_ids", "map", 200));

    debug_visuals_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/homotopy/debug", "map", 200));

    graph_.reset(new Graph(config));
    environment_.Init();

    samples_.resize(config_->n_samples_);
    sample_succes_.resize(config_->n_samples_);

    random_generator_ = RosTools::RandomGenerator(config_->seed_);
    sampling_function_ = &SampleUniformly3D;

    if (config_->topology_comparison_function_ == "UVD")
    {
      topology_comparison_.reset(new UVD());
    }
    else
    {
      topology_comparison_.reset(new Homology());
    }
    debug_benchmarker_.reset(new RosTools::Benchmarker("Homology Comparison"));

    done_ = false;
  }

  void PRM::LoadData(const std::vector<Obstacle> &obstacles, const std::vector<RosTools::Halfspace> &static_obstacles, const Eigen::Vector2d &start, const double orientation,
                     const Eigen::Vector2d &velocity, const std::vector<Goal> &goals, const int previously_selected_id)
  {
    {
      PROFILE_SCOPE("Initializing Obstacles in Environment");

      /* Obstacles */
      environment_.SetPosition(start);
      if (static_obstacles.size() > 0)
        environment_.LoadObstacles(obstacles, static_obstacles);
      else
        environment_.LoadObstacles(obstacles, std::vector<RosTools::Halfspace>({}));
    }

    /* Start */
    start_ = start;
    orientation_ = orientation;
    start_velocity_ = velocity;

    /*for (auto &obstacle : obstacles)
    {
      if ((obstacle.positions_[0] - start_).norm() < obstacle.radius_)
      {
        start_ =
            obstacle.positions_[0] + (start - obstacle.positions_[0]).normalized() * (obstacle.radius_ + 0.05); // Positions [0] is their next position?
      }
    }*/

    /* Goal */
    // Add goals that are collision free
    {
      PROFILE_SCOPE("Setting goals");
      goals_.clear();
      int goal_i = 0;
      for (auto &goal : goals)
      {

        Eigen::Vector2d goal_copy = goal.pos;                       // Need to copy, because goal is const
        environment_.ProjectToFreeSpace(goal_copy, Config::N, 0.5); // Project the goal if it is in collision
        if (environment_.InCollision(SpaceTimePoint(goal_copy(0), goal_copy(1), Config::N)))
        {
          PRM_LOG("Rejecting a goal (it is in collision after projection).");
        }
        else
        {
          goals_.emplace_back(goal_copy, goal.cost);
        }

        goal_i++;
      }
    }

    PRM_LOG(goals_.size() << " Collision-free goals were received");

    /* Other data */
    previously_selected_id_ = previously_selected_id;
  }

  Graph &PRM::Update()
  {
    PROFILE_SCOPE("PRM::Update");
    debug_visuals_->publish(false);

    done_ = false;

    topology_comparison_->Clear();

    graph_->Clear();
    graph_->Initialize(start_, goals_);

    all_samples_.clear();

    next_segment_id_ = -1;

    RosTools::TriggeredTimer prm_timer(config_->timeout_ / 1000.);
    prm_timer.start();

    // Resize, because the number of samples may have been changed
    samples_.resize(config_->n_samples_);
    sample_succes_.resize(config_->n_samples_);

    SampleNewPoints(samples_, sample_succes_); // Draw random samples

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

      // If we only see one goal
      if (visible_goals.size() == 0 && visible_guards.size() == 0)
      {
        AddGuard(i, sample);
        continue;
      }

      // First check if we found a connector, but one that only connects to a single goal
      if (visible_goals.size() <= 1 && visible_goals.size() + visible_guards.size() == 2)
      {
        for (auto &goal : visible_goals)
          visible_guards.push_back(goal); // Add the goal to the visible guards if it exists

        AddSample(i, sample, visible_guards, sample_is_from_previous_iteration); // single threaded
      }
      else if (visible_goals.size() > 1 && visible_guards.size() == 1) // In this case we connect a guard with more than
                                                                       // one goal
      {
        PRM_LOG("New sample connects to more than one goal");

        // There could be more than one homotopy
        Node new_node =
            sample_is_from_previous_iteration ? Node(graph_->GetNodeID(), previous_nodes_[i]) : Node(graph_->GetNodeID(), sample, NodeType::CONNECTOR);

        std::vector<Node *> topology_distinct_goals;
        FindTopologyDistinctGoalConnections(new_node, visible_guards, visible_goals, topology_distinct_goals); /** @todo: resolve slow-down */

        for (auto &goal : topology_distinct_goals) // Check its homotopy agains the other goals that
          AddSample(i, sample, {visible_guards[0], goal}, sample_is_from_previous_iteration);
      }

      // Check for a time-out
      if (prm_timer.hasFinished())
      {
        // PRM_WARN("Timeout on PRM sampling (" << config_->timeout_ << "ms)");
        break;
      }
    }

    done_ = true;
    return *graph_;
  }

  void PRM::SampleNewPoints(std::vector<SpaceTimePoint> &samples, std::vector<bool> &sample_succes)
  {
    // First sample all the points in parallel
    all_samples_.resize(config_->n_samples_);
#pragma omp parallel for num_threads(8)
    for (int i = 0; i < config_->n_samples_; i++)
    {
      sample_succes[i] = true;
      // First resample previous nodes
      bool sample_is_from_previous_iteration = i < (int)previous_nodes_.size();

      // Get a new sample (either from the previous iteration, or a new one)
      SpaceTimePoint sample = sample_is_from_previous_iteration ? previous_nodes_[i].point_ : SampleNewPoint();

      if (config_->visualize_all_samples_)
        all_samples_[i] = sample;

      PRM_LOG("==== [" << i << "] New Sample ====\n"
                       << sample);

      // Check if the sample is in collision
      if (environment_.InCollision(sample))
      {
        PRM_LOG("Sample was in collision. Projecting to free space");
        environment_.ProjectToFreeSpace(sample, 0.1);
        if (environment_.InCollision(sample)) // If that didn't work, then try another sample
          sample_succes[i] = false;
      }

      if (sample_succes[i])
      {
        if (sample_is_from_previous_iteration)
          previous_nodes_[i].point_ = sample; // Update the previous node's position if necessary (for construction later)

        samples[i] = sample;
      }
    }
  }

  void PRM::FindTopologyDistinctGoalConnections(Node &new_node, const std::vector<Node *> &visible_guards, std::vector<Node *> &visible_goals,
                                                std::vector<Node *> &topology_distinct_goals)
  {

    // Sort the costs by how good they are
    std::sort(visible_goals.begin(), visible_goals.end(),
              [&](const Node *goal_a, const Node *goal_b)
              { return Goal::FindGoalWithNode(goals_, goal_a).cost < Goal::FindGoalWithNode(goals_, goal_b).cost; });

    for (auto &goal : visible_goals)
    {
      // Construct the path for this goal
      GeometricPath new_path({visible_guards[0], &new_node, goal});

      // Check the connection
      // if (!ConnectionIsValid(visible_guards[0], goal, new_node.point_))
      if (ConnectionIsValid(new_path))
      {
        PRM_LOG("Found a valid connection to a goal");
        topology_distinct_goals.push_back(goal);
        return;
      }
    }

    /** @note Old implementation: more than one goal per sample */
    /*for (auto &goal : visible_goals)
    {

      if (topology_distinct_goals.size() > 0) // We only accept one path per sample!
        break;

      // Construct the path for this goal
      GeometricPath new_path({visible_guards[0], &new_node, goal});

      // Check the connection
      // if (!ConnectionIsValid(visible_guards[0], goal, new_node.point_))
      if (!ConnectionIsValid(new_path))
      {
        PRM_LOG("The connection to one of the goals is not valid");
        continue;
      }

      bool is_topologically_distinct = true;

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

            other_goal = goal; // Use this goal for the topology

            break;
          }
        }
      }

      if (is_topologically_distinct)
      {
        PRM_LOG("Found a path to the goal with a distinct homotopy");

        topology_distinct_goals.push_back(goal);
      }
    }*/
  }

  void PRM::AddSample(int i, SpaceTimePoint &sample, const std::vector<Node *> guards, bool sample_is_from_previous_iteration)
  {
    PRM_LOG("Guards: " << *guards[0] << " and " << *guards[1]);

    // // Check if the proposed connection is valid
    if (!ConnectionIsValid(guards[0], guards[1], sample))
    {
      PRM_LOG("Sampled connector is not a valid connector");
      return;
    }
    else
      PRM_LOG("Connector is valid");

    Node new_node =
        sample_is_from_previous_iteration ? Node(graph_->GetNodeID(), previous_nodes_[i]) : Node(graph_->GetNodeID(), sample, NodeType::CONNECTOR);
    new_node.type_ = NodeType::CONNECTOR;

    // Get other nodes that connect to these two guards
    std::vector<Node *> shared_neighbours;
    shared_neighbours = graph_->GetSharedNeighbours(guards); // Get all nodes with the same neighbours. The goal guards count as one.
    PRM_LOG("Found " << shared_neighbours.size() << " shared neighbours");

    GeometricPath new_path, other_path;
    if (shared_neighbours.size() > 0)
    {
      new_path = GeometricPath({guards[0], &new_node, guards[1]});

      // // Check if the proposed connection is valid
      // if (!ConnectionIsValid(new_path))
      // {
      //   PRM_LOG("Sampled connector is not a valid connector");
      //   return;
      // }
      // else
      //   PRM_LOG("Connector is valid");
    }

    bool path_is_distinct = true;
    for (auto &neighbour : shared_neighbours)
    {
      ROSTOOLS_ASSERT(neighbour->type_ == NodeType::CONNECTOR, "Shared neighbours should not be guards");
      other_path = GeometricPath({guards[0], neighbour, guards[1]});

      if (AreHomotopicEquivalent(new_path, other_path))
      {
        PRM_LOG("Segment of the new connector is homotopically equivalent to that of " << *neighbour);
        path_is_distinct = false;

        // If they are and the new sample has a shorter path
        if (FirstPathIsBetter(new_path, other_path))
        {
          PRM_LOG("Replacing existing node " << *neighbour << "with faster node " << new_node
                                             << " (difference in length: " << other_path.Length3D() - new_path.Length3D() << " = "
                                             << other_path.Length3D() - new_path.Length3D() / other_path.Length3D() << "%)");
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
      AddNewConnector(new_node, guards);
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
      {
        PropagateNode(node); // (no path argument)
      }
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

    // Then, we would like to propagate this node on its path so that it remains on the path at the same point in time
    // (next iteration) All our current nodes will be the same nodes in the next time step, but one step earlier in time
    if (config_->dynamically_propagate_nodes_)
    {
      propagated_node.point_.Time() = node.point_.Time() - (Config::CONTROL_DT / Config::DT); // Drop by however much discrete steps we
                                                                                              // are moving in one control iteration

      if (propagated_node.point_.Pos()(0) < start_(0)) /** @note Not robust */ //  Do not propagate nodes behind the
                                                                               //  robot
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
          ROSTOOLS_ASSERT(found_first_node, "Did not find the first next node in the path when resampling a previous node");

          auto &next_node = path->nodes_[first_node_id];

          // Sample a new point at its time index (0.5(T2 + T1) / (T_end - T_start)) \in [0, 1]
          SpaceTimePoint new_sample = (*path)((0.5 * (next_node->point_.Time() + node.point_.Time())) /
                                              (path->EndTimeIndex() - path->StartTimeIndex())); // Sample halfway up to the next node

          // Create a node to replace the old
          // Node replacement_node(/*config_->n_samples_ + previous_nodes_.size()*/, new_sample, NodeType::CONNECTOR);
          Node replacement_node(graph_->GetNodeID(), new_sample, NodeType::CONNECTOR);
          replacement_node.SetSegmentAssociation(propagated_node.segment_association_id_); // This may not be homotopically equivalent to the
                                                                                           // previous, but at least this ID is free
          replacement_node.belongs_to_path_ = propagated_node.belongs_to_path_;

          // Remove the previous node and add the replacement
          previous_nodes_.pop_back();
          previous_nodes_.push_back(replacement_node);
        }
        else
        {
          if (propagated_node.type_ != NodeType::CONNECTOR)
            previous_nodes_.pop_back(); // Do not resample GUARDS when they time-out
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
    // bool simple_sampling = true;
    // SpaceTimePoint new_sample(0., 0., 0);
    SpaceTimePoint new_sample = (*sampling_function_)(start_, goals_, random_generator_);

    // if (simple_sampling) // Deprecated
    // {
    // Uniform[0, 10][-5, 5]

    // else
    // {
    //   double start_velocity = start_velocity_.norm(); // Get the forward speed in the non-rotated frame

    //   // Sample a time index
    //   int random_k = random_generator_.Int(Config::N - 2) + 1; // 1 - (N-1)

    //   // Radial sampling based on vehicle limits
    //   // Goal: similar amount of samples per "k", following roughly the actuation limits of the vehicle
    //   // Some settings for the sampler
    //   double view_angle = config_->view_angle_;
    //   double max_velocity = config_->max_velocity_;
    //   double max_acceleration = config_->max_acceleration_;

    //   double maximum_radius = RosTools::dist(start_, goals_.back().pos);

    //   // Find the maximum distance where a node can spawn for the randomly sampled k
    //   double min_vel = start_velocity;
    //   double max_vel = start_velocity;
    //   double max_dist = 0., min_dist = 0.;
    //   for (int i = 0; i < random_k; i++)
    //   {
    //     max_vel = std::min(max_vel + max_acceleration * Config::DT,
    //                        max_velocity); // Either maximum acceleration or maximum velocity
    //     max_dist = std::min(max_dist + max_vel * Config::DT,
    //                         maximum_radius); // The distance we can travel increases with the maximum velocity

    //     // Account for maximum possible braking
    //     min_vel = std::max(min_vel - max_acceleration * Config::DT, 0.);
    //     min_dist = min_dist + min_vel * Config::DT;
    //   }

    //   // Sample u1 such that it spreads quadratically
    //   bool sample_in_quadratic = false;
    //   double u1, u2;
    //   while (!sample_in_quadratic)
    //   {
    //     u1 = random_generator_.Double();
    //     u2 = random_generator_.Double();
    //     if (u2 < u1 * u1) // Rejection sample quadratic distribution
    //       sample_in_quadratic = true;
    //   }

    //   double random_r = min_dist + u1 * (max_dist - min_dist); // Sample a random radius smaller than the velocity
    //   double random_angle = random_generator_.Double() * view_angle - view_angle / 2.;

    //   // Construct the sample
    //   new_sample = SpaceTimePoint(random_r * std::cos(random_angle), random_r * std::sin(random_angle), random_k);

    //   // Check if the sample is acceptable in terms of rotational velocity
    //   double max_w = 1.0;
    //   double cur_psi = 0., cur_x = 0., cur_y = 0.;
    //   double cur_vel = start_velocity;
    //   for (int k = 0; k < random_k; k++)
    //   {
    //     cur_x += cur_vel * std::cos(cur_psi) * Config::DT; // Position
    //     cur_y += cur_vel * std::sin(cur_psi) * Config::DT;
    //     cur_psi += max_w * Config::DT;            // Orientation
    //     cur_vel -= max_acceleration * Config::DT; // Velocity (under maximum braking)

    //     cur_vel = std::max(0., cur_vel);

    //     if (std::abs(cur_psi) > M_PI_2) // Stop when we rotated 90 degrees
    //       break;
    //   }

    //   // If our sample is outside of this rotational velocity area - recursively try again
    //   if ((new_sample(0) < cur_x && new_sample(1) > cur_y) || (new_sample(0) < cur_x && new_sample(1) < -cur_y))
    //     return SampleNewPoint(); // Recursively try again

    // Collapse the range of y values to -max_spread_y, max_spread_y to prevent the view range from becoming excessively
    // large (disabled) double max_spread_y = 5.0; double new_y; if (new_sample(1) > 0)
    //     new_y = new_sample(1) - std::floor(new_sample(1) / max_spread_y) * max_spread_y; // (modulo)
    // else
    //     new_y = new_sample(1) - std::ceil(new_sample(1) / max_spread_y) * max_spread_y; // (modulo)

    // new_sample.SetPos(Eigen::Vector2d(new_sample(0), new_y));

    // Translate and rotate to match the real data
    // }

    /** Translate and rotate to match the real data */
    // Eigen::MatrixXd R = RosTools::rotationMatrixFromHeading(-orientation_);
    // new_sample.SetPos(start_ + /*R * */ new_sample.Pos());

    return new_sample;
  }

  SpaceTimePoint SampleUniformly3D(const Eigen::Vector2d &start, const std::vector<Goal> &goals, RosTools::RandomGenerator &random_generator)
  {
    double extra_range = 4;
    double min_x = std::min(goals.back().pos(0), start(0)) - extra_range;
    double min_y = std::min(goals.back().pos(1), start(1)) - extra_range;
    double range_x = std::max(goals.back().pos(0), start(0)) + extra_range - min_x;
    double range_y = std::max(goals.back().pos(1), start(1)) + extra_range - min_y;
    // double min_x = std::min(goals.back().pos(0) - start(0), 0.) - extra_range;
    // double min_y = std::min(goals.back().pos(1) - start(1), 0.) - extra_range;
    return SpaceTimePoint(min_x + random_generator.Double() * range_x, min_y + random_generator.Double() * range_y,
                          random_generator.Int(Config::N - 2) + 1); // 1 - N-1
  }

  void PRM::FindVisibleGuards(SpaceTimePoint sample, std::vector<Node *> &visible_guards, std::vector<Node *> &visible_goals)
  {
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
    // Add the new node to the graph (note that we are keeping the old one around, but setting its "replaced" flag to
    // true)
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

    PRM_LOG("Adding new guard");
    graph_->AddNode(new_guard); // Add the new guard
  }

  bool PRM::AreHomotopicEquivalent(const GeometricPath &a, const GeometricPath &b)
  {
    debug_benchmarker_->start();
    bool homology_result = topology_comparison_->AreEquivalent(a, b, environment_);
    debug_benchmarker_->stop();

    return homology_result;
  }

  /** @todo: Should be per connection, not all at once */
  bool PRM::ConnectionIsValid(const SpaceTimePoint &first_point, const SpaceTimePoint &second_point)
  {
    // This subfunction checks the connection between any two points

    // Connections must move forward in the "x" direction
    // bool forward_connection =
    //     Eigen::Vector2d(1., 0.).transpose() * RosTools::rotationMatrixFromHeading(orientation_) * second_point.Pos() >
    //     Eigen::Vector2d(1., 0.).transpose() * RosTools::rotationMatrixFromHeading(orientation_) * first_point.Pos(); //.Pos()(0) > first_point.Pos()(0);
    // if (!forward_connection)
    // {
    //   PRM_LOG("Connection does not move forward in time");

    //   return false;
    // }

    // Connections have a limited velocity
    double dist = RosTools::dist(first_point.Pos(), second_point.Pos());
    double vel = dist / ((double)std::abs(first_point.Time() - second_point.Time()) * Config::DT); // The average velocity of this connection
    bool vel_satisfies_limits = vel < config_->max_velocity_;

    if (!vel_satisfies_limits)
    {
      PRM_LOG("Connection does not satisfy velocity limits");

      return false;
    }

    return true; // Connection is valid
  }

  bool PRM::ConnectionIsValid(const GeometricPath &path)
  {
    ROSTOOLS_ASSERT(path.nodes_.size() == 3, "Paths must have 3 nodes to check the connection");

    // First make sure that the connection times are causal
    const Node *start_node = path.nodes_[0];
    const Node *end_node = path.nodes_.back();
    const Node *path_node = path.nodes_[1];
    const SpaceTimePoint &new_point = path_node->point_;
    bool valid = ConnectionIsValid(start_node, end_node, new_point);

    if (!valid)
    {
      PRM_LOG("Connection was invalid");
      return false;
    }

    // Check rules
    // bool rules_satisfied = true;

    // if (config_->pass_left_)
    // {
    //   std::vector<bool> passes_right = topology_comparison_->PassesRight(path, environment_);
    //   for (size_t i = 0; i < passes_right.size(); i++)
    //   {
    //     if (passes_right[i])
    //     {
    //       auto &obstacle = environment_.GetDynamicObstacles()[i];
    //       // if (RosTools::dist(obstacle.positions_[0], path.nodes_[0]->point_.Pos()) > 5.) // Only count if they are close (move this to prm)
    //       // continue;
    //       double angle = std::atan2(obstacle.positions_[1](1) - obstacle.positions_[0](1), obstacle.positions_[1](0) -
    //       obstacle.positions_[0](0)); if (std::abs(angle) > M_PI_2) // Only if they are walking towards the robot?
    //         continue;

    //       rules_satisfied = false;
    //       break;
    //     }
    //   }

    //   if (!rules_satisfied)
    //   {
    //     PRM_LOG("Rules are not satisfied");
    //     return false;
    //   }
    // }

    return true; // This connection is valid
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
      PRM_LOG("Three point connection is not causal (" << start_node->point_.Time() << ", " << new_point.Time() << ", " << end_node->point_.Time()
                                                       << ")");

      return false;
    }

    // Check other conditions per connection
    if (!ConnectionIsValid(start_node->point_, new_point) || !ConnectionIsValid(new_point, end_node->point_))
      return false;

    // ACCELERATIONS
    // Fit time parameterized splines over the points and validate that the accelerations along this spline satisfy the
    // limits
    SpaceTimePoint first_point = start_node->point_;
    SpaceTimePoint second_point = new_point;
    SpaceTimePoint third_point = end_node->point_;

    std::vector<double> t, x, y;
    t.push_back(first_point.Time() * Config::DT);
    t.push_back(second_point.Time() * Config::DT);
    t.push_back(third_point.Time() * Config::DT);

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
    // Prefer goals
    double goal_1_cost, goal_2_cost;
    if (first_path.nodes_.back()->type_ == NodeType::GOAL && second_path.nodes_.back()->type_ == NodeType::GOAL)
    {
      // Bit annoying: Find which goals they are to find the associated costs
      goal_1_cost = Goal::FindGoalWithNode(goals_, first_path.nodes_.back()).cost;
      goal_2_cost = Goal::FindGoalWithNode(goals_, second_path.nodes_.back()).cost;

      if (goal_1_cost != goal_2_cost)
        return goal_1_cost < goal_2_cost; // Is the goal better?
      else
        return first_path.RelativeSmoothness() < second_path.RelativeSmoothness(); // Is it smoother
    }
    return first_path.RelativeSmoothness() < second_path.RelativeSmoothness();
  }

  void PRM::Reset()
  {
    PRM_LOG("Reset()");

    done_ = false;
    config_->seed_ += 1; // Keep the randomizer consistent for every experiment
    random_generator_ = RosTools::RandomGenerator(config_->seed_);

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
    RosTools::ROSPointMarker &sphere = ros_graph_visuals_->getNewPointMarker("SPHERE");
    sphere.setScale(0.3, 0.3, 0.3);

    RosTools::ROSLine &edge = ros_graph_visuals_->getNewLine();
    edge.setScale(0.1, 0.1);
    edge.setColor(0., 0., 0., 1.0);

    RosTools::ROSTextMarker &segment_text = ros_segment_visuals_->getNewTextMarker();
    segment_text.setScale(1.0);

    int num_guards = 0;
    int num_connectors = 0;
    for (auto &node : graph_->nodes_)
    {
      ROSTOOLS_ASSERT(node.type_ != NodeType::NONE, "Node type needs to be defined for all nodes.");
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
      RosTools::ROSPointMarker &samples = ros_sample_visuals_->getNewPointMarker("SPHERE");
      samples.setScale(.15, .15, .15);
      samples.setColorInt(0);

      for (auto &sample : all_samples_)
        samples.addPointMarker(sample.MapToTime());
    }

    ros_sample_visuals_->publish();
  }
} // namespace Homotopy
