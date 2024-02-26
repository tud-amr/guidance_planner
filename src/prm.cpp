#include "guidance_planner/prm.h"

#include <ros_tools/profiling.h>
#include <ros_tools/data_saver.h>

namespace GuidancePlanner
{

  PRM::~PRM() {}

  PRM::PRM() {}

  void PRM::Init(ros::NodeHandle &nh, Config *config)
  {
    config_ = config;

    ros_sample_visuals_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/homotopy/all_samples", "map", 500));
    ros_graph_visuals_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/homotopy/graph", "map", 200));
    ros_goal_start_visuals_.reset(new RosTools::ROSMarkerPublisher(nh, "guidance_planner/start_and_goals", "map", 50));
    ros_segment_visuals_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/homotopy/segment_ids", "map", 200));

    debug_visuals_.reset(new RosTools::ROSMarkerPublisher(nh, "lmpcc/homotopy/debug", "map", 200));

    graph_.reset(new Graph(config));
    environment_.Init();

    samples_.resize(config_->n_samples_);
    sample_succes_.resize(config_->n_samples_);

    random_generator_ = RosTools::RandomGenerator(config_->seed_);

    if (config_->sampling_function_ == "Uniform")
      sampling_function_ = &PRM::SampleUniformly3D;

    if (config_->topology_comparison_function_ == "UVD")
    {
      topology_comparison_.reset(new UVD());
    }
    else if (config_->topology_comparison_function_ == "None")
    {
      topology_comparison_.reset(new NoTopologyComparison());
    }
    else
    {
      topology_comparison_.reset(new Homology(nh, config_->assume_constant_velocity_));
    }
    // debug_benchmarker_.reset(new RosTools::Benchmarker("Homology Comparison"));

    done_ = false;
  }

  void PRM::LoadData(const std::vector<Obstacle> &obstacles, const std::vector<RosTools::Halfspace> &static_obstacles, const Eigen::Vector2d &start, const double orientation,
                     const Eigen::Vector2d &velocity, const std::vector<Goal> &goals)
  {
    {
      PROFILE_SCOPE("Initializing Obstacles in Environment");
      PRM_LOG("Loading data into PRM...")

      /* Obstacles */
      environment_.SetPosition(start);
      PRM_LOG("Static obstacles size: " << static_obstacles.size());
      environment_.LoadObstacles(obstacles, static_obstacles);
    }

    /* Start */
    start_ = start;
    orientation_ = orientation;
    start_velocity_ = velocity;

    SpaceTimePoint start_point(start_(0), start_(1), 0.);
    environment_.ProjectToFreeSpace(start_point, 0.1);
    start_(0) = start_point.Pos()(0);
    start_(1) = start_point.Pos()(1);

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

    // Compute the range of the planning space (between start and goals)
    double min_x = start_(0), max_x = start_(0), min_y = start_(1), max_y = start_(1);
    for (auto &goal : goals_)
    {
      min_x = std::min(goal.pos(0), min_x);
      min_y = std::min(goal.pos(1), min_y);

      max_x = std::max(goal.pos(0), max_x);
      max_y = std::max(goal.pos(1), max_y);
    }
    range_x_ = max_x - min_x + config_->sample_margin_;
    range_y_ = max_y - min_y + config_->sample_margin_;
    min_x_ = min_x - config_->sample_margin_ / 2.;
    min_y_ = min_y - config_->sample_margin_ / 2.;

    if (std::abs(range_x_) < 1e-3 && config_->sample_margin_ == 0.)
      ROS_WARN("The x range of sampling is zero (goal and start on a line) please use config_->sample_margin_ > 0.");
    if (std::abs(range_y_) < 1e-3 && config_->sample_margin_ == 0.)
      ROS_WARN("The y range of sampling is zero please use config_->sample_margin_ > 0.");
  }

  Graph &PRM::Update()
  {
    PROFILE_SCOPE("PRM::Update");
    PRM_LOG("PRM::Update")

    debug_visuals_->publish(false);

    done_ = false;

    topology_comparison_->Clear();
    graph_->Clear();
    all_samples_.clear();

    RosTools::TriggeredTimer prm_timer(config_->timeout_ / 1000.);
    prm_timer.start();

    graph_->Initialize(start_, goals_);

    // Resize, because the number of samples may have been changed
    samples_.resize(config_->n_samples_);
    sample_succes_.resize(config_->n_samples_);

    SampleNewPoints(samples_, sample_succes_); // Draw random samples
    PRM_LOG("New candidate nodes ready. Inserting them into the Visibility-PRM graph");

    // Then add them to the graph
    for (int i = 0; i < config_->n_samples_; i++)
    {
      if (!sample_succes_[i])
        continue;

      if (prm_timer.hasFinished()) // Timeout
      {
        // PRM_WARN("Timeout on PRM sampling (" << config_->timeout_ << "ms)");
        break;
      }

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
      else if (visible_goals.size() > 1 && visible_guards.size() == 1) // We connect a guard with more than one goal
      {
        PRM_LOG("New sample connects to more than one goal");
        Node new_node = sample_is_from_previous_iteration ? Node(graph_->GetNodeID(), previous_nodes_[i])
                                                          : Node(graph_->GetNodeID(), sample, NodeType::CONNECTOR);

        Node *goal = FindTopologyDistinctGoalConnection(new_node, visible_guards, visible_goals);
        if (goal != nullptr)
          AddSample(i, sample, {visible_guards[0], goal}, sample_is_from_previous_iteration);
      }
    }

    PRM_LOG("Visibility-PRM Graph Done.");

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

      bool sample_is_from_previous_iteration = i < (int)previous_nodes_.size(); // First resample previous nodes

      // Get a new sample (either from the previous iteration, or a new one)
      SpaceTimePoint sample = sample_is_from_previous_iteration ? previous_nodes_[i].point_ : SampleNewPoint();

      if (environment_.InCollision(sample)) // Check if the sample is in collision
      {
        PRM_LOG("Sample was in collision. Projecting to free space");
        environment_.ProjectToFreeSpace(sample, 0.1);
        if (environment_.InCollision(sample))
          sample_succes[i] = false; // If that didn't work, then try another sample
      }

      if (sample_succes[i])
      {
        if (sample_is_from_previous_iteration)
          previous_nodes_[i].point_ = sample; // Update the previous node's position if necessary (for construction later)

        samples[i] = sample;
      }

      if (config_->visualize_all_samples_)
        all_samples_[i] = sample;
    }
  }

  Node *PRM::FindTopologyDistinctGoalConnection(Node &new_node, const std::vector<Node *> &visible_guards, std::vector<Node *> &visible_goals)
  {
    // Sort the costs to have the best goals initially
    std::sort(visible_goals.begin(), visible_goals.end(), [&](const Node *goal_a, const Node *goal_b)
              { return Goal::FindGoalWithNode(goals_, goal_a).cost < Goal::FindGoalWithNode(goals_, goal_b).cost; });

    for (auto &goal : visible_goals)
    {
      GeometricPath new_path({visible_guards[0], &new_node, goal}); // Construct the path for this goal
      if (ConnectionIsValid(new_path))                              // Check validity
        return goal;
    }

    return nullptr;
  }

  void PRM::AddSample(int i, SpaceTimePoint &sample, const std::vector<Node *> guards, bool sample_is_from_previous_iteration)
  {
    PRM_LOG("Guards: " << *guards[0] << " and " << *guards[1]);

    if (!ConnectionIsValid(guards[0], guards[1], sample)) // Check if the proposed connection is valid
    {
      PRM_LOG("Sampled connector is not a valid connector");
      return;
    }
    else
      PRM_LOG("Connector is valid");

    Node new_node = sample_is_from_previous_iteration
                        ? Node(graph_->GetNodeID(), previous_nodes_[i])
                        : Node(graph_->GetNodeID(), sample, NodeType::CONNECTOR);
    new_node.type_ = NodeType::CONNECTOR;

    std::vector<Node *> shared_neighbours;
    shared_neighbours = graph_->GetSharedNeighbours(guards); // Get all nodes with the same neighbours. The goal guards count as one.
    PRM_LOG("Found " << shared_neighbours.size() << " shared neighbours");

    GeometricPath new_path, other_path;
    if (shared_neighbours.size() > 0)
      new_path = GeometricPath({guards[0], &new_node, guards[1]});

    bool path_is_distinct = true;
    for (auto &neighbour : shared_neighbours)
    {
      ROSTOOLS_ASSERT(neighbour->type_ == NodeType::CONNECTOR, "Shared neighbours should not be guards");
      other_path = GeometricPath({guards[0], neighbour, guards[1]});

      if (AreHomotopicEquivalent(new_path, other_path))
      {
        PRM_LOG("Segment of the new connector is homotopically equivalent to that of " << *neighbour);
        path_is_distinct = false;

        if (FirstPathIsBetter(new_path, other_path, config_->min_path_improvement_))
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

    if (path_is_distinct)
      AddNewConnector(new_node, guards);
  }

  void PRM::PropagateGraph(const std::vector<GeometricPath> &paths)
  {
    previous_nodes_.clear();
    for (auto &node : graph_->nodes_)
    {
      if (node.replaced_ || node.id_ < 0) // Do not consider replaced nodes or the start/goal
        continue;

      const GeometricPath *node_path = nullptr; // To which path does this node belong
      if (node.type_ == NodeType::CONNECTOR)    // For all connectors
      {
        for (size_t p = 0; p < paths.size(); p++)
        {
          if (paths[p].ContainsNode(node))
          {
            node_path = &(paths[p]);
            break;
          }
        }
      }

      PropagateNode(node, node_path);
    }
  }

  void PRM::PropagateNode(const Node &node, const GeometricPath *path)
  {
    previous_nodes_.push_back(node); // Copy the given node to save it (by value, because the graph will be reset)
    auto &propagated_node = previous_nodes_.back();

    // Then, we would like to propagate this node on its path so that it remains on the path at the same point in time
    // (next iteration) All our current nodes will be the same nodes in the next time step, but one step earlier in time
    if (config_->dynamically_propagate_nodes_)
    {
      propagated_node.point_.Time() = node.point_.Time() - (Config::CONTROL_DT / Config::DT); // Drop by however much discrete steps we
                                                                                              // are moving in one control iteration

      // if (propagated_node.point_.Pos()(0) < start_(0))
      // {
      // previous_nodes_.pop_back();
      // return;
      // }

      // If a node hits the floor when it belongs to a path, then we should resample to ensure that the path remains valid
      if (propagated_node.point_.Time() < 1)
      {
        if (propagated_node.type_ == NodeType::CONNECTOR && path != nullptr) // Resample connectors
        {
          // Find the first node in the path that is not our current node
          // bool found_first_node = false;
          // int first_node_id = 1; // Skip the start
          // for (; first_node_id < (int)path->nodes_.size(); first_node_id++)
          // {
          //   if (path->nodes_[first_node_id]->id_ != node.id_)
          //   {
          //     found_first_node = true;
          //     break;
          //   }
          // }
          // ROSTOOLS_ASSERT(found_first_node, "Did not find the first next node in the path when resampling a previous node");
          int first_node_id = 2; // Cannot be the start, cannot be the first connector
          auto &next_node = path->nodes_[first_node_id];

          // Sample halfway up to the next node (0.5(T2 + T1) / (T_end - T_start)) \in [0, 1]
          propagated_node.point_ = (*path)((0.5 * (next_node->point_.Time() + node.point_.Time())) /
                                           (path->EndTimeIndex() - path->StartTimeIndex()));

          // Node replacement_node(graph_->GetNodeID(), new_sample, NodeType::CONNECTOR); // Replace the node
          // replacement_node.belongs_to_path_ = propagated_node.belongs_to_path_;

          // // Remove the previous node and add the replacement
          // previous_nodes_.pop_back();
          // previous_nodes_.push_back(replacement_node);
        }
        else
        {
          if (propagated_node.type_ != NodeType::CONNECTOR)
            previous_nodes_.pop_back(); // Do not resample GUARDS when they time-out
        }
      }
    }
  }

  SpaceTimePoint PRM::SampleNewPoint() { return (this->*sampling_function_)(); }
  SpaceTimePoint PRM::SampleUniformly3D()
  {
    return SpaceTimePoint(min_x_ + random_generator_.Double() * range_x_,
                          min_y_ + random_generator_.Double() * range_y_,
                          random_generator_.Int(Config::N - 2) + 1);
  }

  void PRM::FindVisibleGuards(SpaceTimePoint sample, std::vector<Node *> &visible_guards, std::vector<Node *> &visible_goals)
  {
    for (Node &node : graph_->nodes_)
    {
      if (node.type_ == NodeType::GUARD || node.type_ == NodeType::GOAL)
      {
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
    // Add the new node to the graph (note that we are keeping the old one around, but setting its "replaced" flag to true)
    Node *new_node_ptr = graph_->AddNode(new_node);

    // Set its neighbours
    new_node_ptr->neighbours_.push_back(visible_guards[0]);
    new_node_ptr->neighbours_.push_back(visible_guards[1]);

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
    // debug_benchmarker_->start();
    bool homology_result = topology_comparison_->AreEquivalent(a, b, environment_);
    // debug_benchmarker_->stop();

    return homology_result;
  }

  double PRM::GetHomotopicCost(const GeometricPath &a, const GeometricPath &b)
  {
    // debug_benchmarker_->start();
    double homology_cost = reinterpret_cast<Homology *>(topology_comparison_.get())->GetCost(a, b, environment_);
    // debug_benchmarker_->stop();

    return homology_cost;
  }

  std::vector<bool> PRM::PassesRight(const GeometricPath &path)
  {
    // debug_benchmarker_->start();
    std::vector<bool> h = topology_comparison_->LeftPassingVector(path, environment_);
    // debug_benchmarker_->stop();

    return h;
  }

  /** @todo: Should be per connection, not all at once */
  bool PRM::ConnectionIsValid(const SpaceTimePoint &first_point, const SpaceTimePoint &second_point)
  {
    // This subfunction checks the connection between any two points
    // Connections must move forward in the "x" direction
    bool forward_connection = (!config_->enable_forward_filter_) || (Eigen::Vector2d(1., 0.).transpose() * RosTools::rotationMatrixFromHeading(orientation_) * second_point.Pos() >
                                                                     Eigen::Vector2d(1., 0.).transpose() * RosTools::rotationMatrixFromHeading(orientation_) * first_point.Pos()); //.Pos()(0) > first_point.Pos()(0);
    if (!forward_connection)
    {
      PRM_LOG("Connection does not move forward in time");
      return false;
    }

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
    if (config_->enable_acceleration_filter_)
    {
      // Fit time parameterized splines over the points and validate that the accelerations along this spline satisfy the limits
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
    }

    return true; // This connection is valid
  }

  bool PRM::FirstPathIsBetter(const GeometricPath &first_path, const GeometricPath &second_path, double min_improvement)
  {
    double goal_1_cost, goal_2_cost;
    if (first_path.nodes_.back()->type_ == NodeType::GOAL && second_path.nodes_.back()->type_ == NodeType::GOAL)
    {
      // Bit annoying: Find which goals they are to find the associated costs
      goal_1_cost = Goal::FindGoalWithNode(goals_, first_path.nodes_.back()).cost;
      goal_2_cost = Goal::FindGoalWithNode(goals_, second_path.nodes_.back()).cost;

      if (goal_1_cost != goal_2_cost)
        return goal_1_cost < goal_2_cost; // Is the goal better?
    }

    return first_path.RelativeSmoothness() < second_path.RelativeSmoothness();
  }

  void PRM::Reset()
  {
    PRM_LOG("Reset()");

    done_ = false;
    config_->seed_ += 1; // Keep the randomizer consistent for every experiment
    random_generator_ = RosTools::RandomGenerator(config_->seed_);

    previous_nodes_.clear(); // Forget nodes
  }

  void PRM::Visualize()
  {
    VisualizeGraph();
    VisualizeAllSamples();

    debug_visuals_->publish(true);

    if (config_->visualize_homology_)
      topology_comparison_->Visualize(environment_);
  }

  void PRM::VisualizeGraph()
  {
    // NODES IN THE GRAPH - COLORED BY PATH / TYPE
    RosTools::ROSPointMarker &sphere = ros_graph_visuals_->getNewPointMarker("SPHERE");
    sphere.setScale(0.1, 0.1, 0.1);

    RosTools::ROSPointMarker &goal_start_sphere = ros_goal_start_visuals_->getNewPointMarker("SPHERE");
    goal_start_sphere.setScale(0.1, 0.1, 0.1);

    RosTools::ROSLine &edge = ros_graph_visuals_->getNewLine();
    edge.setScale(0.05, 0.05);
    edge.setColor(0., 0., 0., 1.);

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
        {
          goal_start_sphere.setColor(249. / 256., 142. / 256., 9. / 256., 1.); // Start & End coloring
          goal_start_sphere.addPointMarker(node.point_.MapToTime());
        }
        else
        {
          sphere.setColor(249. / 256., 142. / 256., 9. / 256., 1.); // Guard coloring
          sphere.addPointMarker(node.point_.MapToTime());
        }
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

          if (node.belongs_to_path_ >= 0)
          {
            segment_text.setText(std::to_string(node.belongs_to_path_));
            segment_text.setColorInt(node.belongs_to_path_, 20);
            segment_text.addPointMarker(node_pose + Eigen::Vector3d(0., 0.5, 0.5));
          }
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
    ros_goal_start_visuals_->publish();
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

      for (size_t s = 0; s < all_samples_.size(); s++)
      {
        if (sample_succes_[s])
          samples.addPointMarker(all_samples_[s].MapToTime());
      }
    }

    ros_sample_visuals_->publish();
  }
} // namespace Homotopy
