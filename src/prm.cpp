#include "guidance_planner/prm.h"

#include <guidance_planner/graph.h>

#include <guidance_planner/environment.h>
#include <guidance_planner/sampler.h>

#include <guidance_planner/homotopy_comparison/homology.h>
#include <guidance_planner/homotopy_comparison/uvd.h>
#include <guidance_planner/homotopy_comparison/winding_angle.h>

#include <guidance_planner/utils.h>

#include <ros_tools/profiling.h>
#include <ros_tools/math.h>
#include <ros_tools/data_saver.h>
#include <ros_tools/visuals.h>

namespace GuidancePlanner
{

  PRM::~PRM() {}

  PRM::PRM() {}

  void PRM::Init(Config *config)
  {
    config_ = config;

    graph_.reset(new Graph(config));
    environment_ = std::make_shared<Environment>();
    environment_->Init();

    sampler_ = std::make_shared<Sampler>(config_);

    if (Config::use_dubins_path_)
      LOG_VALUE("Connection Type", "Dubins Path");
    else
      LOG_VALUE("Connection Type", "Straight Path");

    if (config_->topology_comparison_function_ == "UVD")
    {
      topology_comparison_.reset(new UVD());
      LOG_VALUE("Topology Comparison", "UVD");
    }
    else if (config_->topology_comparison_function_ == "None")
    {
      topology_comparison_.reset(new NoHomotopyComparison());
      LOG_VALUE("Topology Comparison", "None");
    }
    else if (config_->topology_comparison_function_ == "Winding")
    {
      topology_comparison_.reset(new WindingAngle());
      ((WindingAngle *)topology_comparison_.get())->pass_threshold_ = config_->winding_pass_threshold_;
      LOG_VALUE("Topology Comparison", "Winding Angles");
    }
    else
    {
      topology_comparison_.reset(new Homology(config_->assume_constant_velocity_));
      LOG_VALUE("Topology Comparison", "Homology");
    }

    done_ = false;
  }

  void PRM::LoadData(const std::vector<Obstacle> &obstacles, const std::vector<Halfspace> &static_obstacles, const Eigen::Vector2d &start, const double orientation,
                     const Eigen::Vector2d &velocity, const std::vector<Goal> &goals)
  {
    {
      PROFILE_SCOPE("Initializing Obstacles in Environment");
      PRM_LOG("Loading data into PRM...")

      /* Obstacles */
      environment_->SetPosition(start);
      PRM_LOG("Static obstacles size: " << static_obstacles.size());
      environment_->LoadObstacles(obstacles, static_obstacles);
    }

    /* Start */
    start_ = start;
    orientation_ = orientation;
    start_velocity_ = velocity;

    SpaceTimePoint start_point(start_(0), start_(1), 0.);
    environment_->ProjectToFreeSpace(start_point, 0.1);
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

        Eigen::Vector2d goal_copy = goal.pos;                        // Need to copy, because goal is const
        environment_->ProjectToFreeSpace(goal_copy, Config::N, 0.5); // Project the goal if it is in collision
        if (environment_->InCollision(SpaceTimePoint(goal_copy(0), goal_copy(1), Config::N)))
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

    // Sort the costs to have the best goals initially
    std::sort(goals_.begin(), goals_.end(), [&](const Goal goal_a, const Goal goal_b)
              { return goal_a.cost < goal_b.cost; });

    PRM_LOG(goals_.size() << " Collision-free goals were received");

    sampler_->SetRange(start_, goals_);
  }

  Graph &PRM::Update()
  {
    PROFILE_SCOPE("PRM::Update");
    PRM_LOG("PRM::Update")
    done_ = false;

    BENCHMARKERS.getBenchmarker("homotopy_comparison").reset();

    topology_comparison_->Clear();
    graph_->Clear();
    sampler_->Clear();

    RosTools::Timer prm_timer(config_->timeout_ / 1000.);
    prm_timer.start();

    graph_->Initialize(start_, goals_);

    SampleNewPoints(); // Draw random samples
    PRM_LOG("New candidate nodes ready. Inserting them into the Visibility-PRM graph");

    // Then add them to the graph
    for (int i = 0; i < config_->n_samples_; i++)
    {
      Sample &sample = sampler_->GetSample(i);

      if (!sample.success)
        continue;

      if (prm_timer.hasFinished()) // Timeout
      {
        // PRM_WARN("Timeout on PRM sampling (" << config_->timeout_ << "ms)");
        break;
      }

      bool sample_is_from_previous_iteration = i < (int)previous_nodes_.size();

      // Find the number of visible guards from this node
      std::vector<Node *> visible_guards;
      FindVisibleGuards(sample.point, visible_guards);

      // Find out if at least one goal is visible, if it is, save it
      Node *goal_node = nullptr;
      int goal_index = 0;
      for (size_t g = 0; g < graph_->goal_nodes_.size(); g++)
      {
        if (IsGoalVisible(sample.point, g))
        {
          goal_node = graph_->goal_nodes_[g];
          goal_index = g;
          break;
        }
      }

      bool goal_visible = goal_node != nullptr;

      if (goal_visible)
      {
        PRM_LOG(visible_guards.size() << " guards and a goal visible");
      }
      else
      {
        PRM_LOG(visible_guards.size() << " guards and no goals visible");
      }

      // CREATE A GUARD: If we see no goals and no guards
      if (!goal_visible && visible_guards.size() == 0)
      {
        AddGuard(i, sample.point);
        continue;
      } // CREATE A CONNECTOR: If we found one guard and at least one goal
      else if (visible_guards.size() == 2 && !goal_visible)
      {
        AddSample(i, sample.point, visible_guards, sample_is_from_previous_iteration);
      }
      else if (goal_visible && visible_guards.size() == 1) // visible_goals.size() <= 1 && visible_goals.size() + visible_guards.size() == 2)
      {

        Node new_node = sample_is_from_previous_iteration ? Node(graph_->GetNodeID(), previous_nodes_[i])
                                                          : Node(graph_->GetNodeID(), sample.point, NodeType::CONNECTOR);

        Node *valid_goal = nullptr;
        int valid_goal_index = 0;
        for (size_t g = goal_index; g < graph_->goal_nodes_.size() && valid_goal == nullptr; g++)
        {
          if (g != goal_index && !IsGoalVisible(sample.point, g))
            continue;

          valid_goal = CheckGoalConnection(new_node, visible_guards[0], graph_->goal_nodes_[g]);
          valid_goal_index = g;
        }

        if (valid_goal == nullptr) // Add a connector if there was a valid goal
          continue;

        visible_guards.push_back(valid_goal);

        AddSample(i, sample.point, visible_guards, sample_is_from_previous_iteration); // single threaded

        // Swap goals if there are equal cost goals, to make the graph more robust
        if (valid_goal_index == graph_->goal_nodes_.size() - 1)
          continue;

        if (Goal::FindGoalWithNode(goals_, graph_->goal_nodes_[valid_goal_index + 1]).cost == Goal::FindGoalWithNode(goals_, graph_->goal_nodes_[valid_goal_index]).cost)
        {
          // Swap the goals
          auto *temp = graph_->goal_nodes_[valid_goal_index];
          graph_->goal_nodes_[valid_goal_index] = graph_->goal_nodes_[valid_goal_index + 1];
          graph_->goal_nodes_[valid_goal_index + 1] = temp;
        }
      }
    }

    PRM_LOG("Visibility-PRM Graph Done.");

    done_ = true;
    return *graph_;
  }

  // IsGoalValid
  Node *PRM::CheckGoalConnection(Node &new_node, Node *guard, Node *goal) const
  {
    GeometricPath new_path({guard, &new_node, goal}); // Construct the path for this goal

    if (new_path.isValid(config_, start_velocity_, orientation_))
      return goal;

    return nullptr;
  }

  void PRM::SampleNewPoints()
  {
// First sample all the points in parallel
#pragma omp parallel for num_threads(8)
    for (int i = 0; i < config_->n_samples_; i++)
    {
      bool sample_is_from_previous_iteration = i < (int)previous_nodes_.size(); // First resample previous nodes

      // Get a new sample (either from the previous iteration, or a new one)
      // Sample &sample = sample_is_from_previous_iteration ? prev_node : sampler_->SampleUniformly(i);
      Sample &sample = sampler_->SampleUniformly(i);
      if (sample_is_from_previous_iteration)
        sample.point = previous_nodes_[i].point_;

      if (environment_->InCollision(sample.point)) // Check if the sample is in collision
      {
        PRM_LOG("Sample was in collision. Projecting to free space");
        environment_->ProjectToFreeSpace(sample.point, 0.1);
        if (environment_->InCollision(sample.point))
          sample.success = false; // If that didn't work, then try another sample
      }

      if (sample.success)
      {
        if (sample_is_from_previous_iteration)
          previous_nodes_[i].point_ = sample.point; // Update the previous node's position if necessary (for construction later)
      }
    }
  }

  void PRM::AddSample(int i, SpaceTimePoint &sample, const std::vector<Node *> guards, bool sample_is_from_previous_iteration)
  {
    PRM_LOG("Guards: " << *guards[0] << " and " << *guards[1]);

    // if (!ConnectionIsValid(guards[0], guards[1], sample)) // Check if the proposed connection is valid
    Node temporary_node(-1e2, sample, NodeType::CONNECTOR);                     // temporary new node
    int g0_index = guards[0]->point_.Time() < guards[1]->point_.Time() ? 0 : 1; // Which guard is first
    int g1_index = g0_index == 0 ? 1 : 0;
    GeometricPath temporary_path({guards[g0_index], &temporary_node, guards[g1_index]});

    if (!temporary_path.isValid(config_, start_velocity_, orientation_)) // Check if the proposed connection is valid
      return;

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
    do_not_propagate_nodes_ = false;
  }

  void PRM::PropagateNode(const Node &node, const GeometricPath *path)
  {
    previous_nodes_.push_back(node); // Copy the given node to save it (by value, because the graph will be reset)

    if (!config_->dynamically_propagate_nodes_) // Setting must be enabled in general
      return;

    if (do_not_propagate_nodes_) // This setting can be enabled externally to stop node propagation for one iteration
      return;

    auto &propagated_node = previous_nodes_.back();

    // Then, we would like to propagate this node on its path so that it remains on the path at the same point in time
    // (next iteration) All our current nodes will be the same nodes in the next time step, but one step earlier in time

    propagated_node.point_.Time() = node.point_.Time() - (Config::CONTROL_DT / Config::DT); // Drop by however much discrete steps we
                                                                                            // are moving in one control iteration

    // If a node hits the floor when it belongs to a path, then we should resample to ensure that the path remains valid
    if (propagated_node.point_.Time() < 1)
    {
      if (propagated_node.type_ == NodeType::CONNECTOR && path != nullptr) // Resample connectors
      {

        int first_node_id = 2; // Cannot be the start, cannot be the first connector
        auto *next_node = path->GetConnections()[0]->getEnd();

        // Sample halfway up to the next node (0.5(T2 + T1) / (T_end - T_start)) \in [0, 1]
        propagated_node.point_ = (*path)((0.5 * (next_node->point_.Time() + node.point_.Time())) /
                                         (path->EndTimeIndex() - path->StartTimeIndex()));
      }
      else
      {
        if (propagated_node.type_ != NodeType::CONNECTOR)
          previous_nodes_.pop_back(); // Do not resample GUARDS when they time-out
      }
    }
  }

  bool PRM::IsGoalVisible(SpaceTimePoint sample, int goal_index) const
  {
    Node *goal_node = graph_->goal_nodes_[goal_index];
    return environment_->IsVisible(sample, goal_node->point_);
  }

  void PRM::FindVisibleGuards(SpaceTimePoint sample, std::vector<Node *> &visible_guards)
  {
    for (Node &node : graph_->nodes_)
    {
      if (node.type_ == NodeType::GUARD)
      {
        if (environment_->IsVisible(sample, node.point_))
          visible_guards.push_back(&node);
      }
    }
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
    if (environment_->InCollision(sample, 0.1))
      return;

    PRM_LOG("Adding new guard");
    graph_->AddNode(new_guard); // Add the new guard
  }

  bool PRM::AreHomotopicEquivalent(const GeometricPath &a, const GeometricPath &b)
  {
    BENCHMARKERS.getBenchmarker("homotopy_comparison").start();
    bool homology_result = topology_comparison_->AreEquivalent(a, b, *environment_);
    BENCHMARKERS.getBenchmarker("homotopy_comparison").stop();

    return homology_result;
  }

  double PRM::GetHomotopicCost(const GeometricPath &a, const GeometricPath &b)
  {
    // debug_benchmarker_->start();
    double homology_cost = reinterpret_cast<Homology *>(topology_comparison_.get())->GetCost(a, b, *environment_);
    // debug_benchmarker_->stop();

    return homology_cost;
  }

  std::vector<bool> PRM::PassesRight(const GeometricPath &path)
  {
    // debug_benchmarker_->start();
    std::vector<bool> h = topology_comparison_->LeftPassingVector(path, *environment_);
    // debug_benchmarker_->stop();

    return h;
  }

  std::vector<bool> PRM::GetLeftPassingVector(const GeometricPath &path)
  {
    return topology_comparison_->LeftPassingVector(path, *environment_);
  }

  bool PRM::FirstPathIsBetter(const GeometricPath &first_path, const GeometricPath &second_path)
  {
    double goal_1_cost, goal_2_cost;
    if (first_path.GetEnd()->type_ == NodeType::GOAL && second_path.GetEnd()->type_ == NodeType::GOAL)
    {
      // Bit annoying: Find which goals they are to find the associated costs
      goal_1_cost = Goal::FindGoalWithNode(goals_, first_path.GetEnd()).cost;
      goal_2_cost = Goal::FindGoalWithNode(goals_, second_path.GetEnd()).cost;

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
    sampler_->Reset();

    previous_nodes_.clear(); // Forget nodes
  }

  void PRM::Visualize()
  {
    VisualizeGraph();

    sampler_->Visualize();

    if (config_->visualize_homology_)
      topology_comparison_->Visualize(*environment_);
  }

  void PRM::VisualizeGraph()
  {
    // NODES IN THE GRAPH - COLORED BY PATH / TYPE
    double sphere_size = 0.15;
    auto &graph_visuals = VISUALS.getPublisher("guidance_planner/graph");
    auto &sphere = graph_visuals.getNewPointMarker("SPHERE");
    sphere.setScale(sphere_size, sphere_size, sphere_size);

    auto &edge = graph_visuals.getNewLine();
    edge.setScale(0.05, 0.05);
    edge.setColor(0., 0., 0., 1.);

    auto &start_goal_visuals = VISUALS.getPublisher("guidance_planner/start_and_goals");
    auto &goal_start_sphere = start_goal_visuals.getNewPointMarker("SPHERE");
    goal_start_sphere.setScale(sphere_size, sphere_size, sphere_size);

    auto &segments_visuals = VISUALS.getPublisher("guidance_planner/segments");
    auto &segment_text = segments_visuals.getNewTextMarker();
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

    graph_visuals.publish();
    start_goal_visuals.publish();
    segments_visuals.publish();
  }

  void PRM::saveData(RosTools::DataSaver &data_saver)
  {
    data_saver.AddData("homotopy_comparison_runtime", BENCHMARKERS.getBenchmarker("homotopy_comparison").getTotalDuration());
  }

} // namespace GuidancePlanner
