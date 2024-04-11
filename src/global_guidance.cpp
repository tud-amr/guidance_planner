#include "guidance_planner/global_guidance.h"

#include <guidance_planner/config.h>
#include <guidance_planner/cubic_spline.h>
#include <guidance_planner/paths.h>
#include <guidance_planner/graph.h>
#include <guidance_planner/utils.h>

#include <ros_tools/profiling.h>
#include <ros_tools/data_saver.h>

#include <omp.h>

namespace GuidancePlanner
{
  GlobalGuidance::OutputTrajectory::OutputTrajectory(const GeometricPath &_path, const CubicSpline3D &_spline)
  {
    path = StandaloneGeometricPath(_path);
    for (auto &node : path.saved_nodes_)
    {
      if (node.type_ == NodeType::CONNECTOR && node.id_ < 1e4)
        node.id_ += 1e4;
    }

    spline = _spline;

    topology_class = -1;
    previously_selected_ = false;
    color_ = -1;
  }

  GlobalGuidance::OutputTrajectory::OutputTrajectory(const GlobalGuidance::OutputTrajectory &other)
  {
    path = StandaloneGeometricPath(other.path);
    for (auto &node : path.saved_nodes_)
    {
      if (node.type_ == NodeType::CONNECTOR && node.id_ < 1e4)
        node.id_ += 1e4;
    }

    spline = other.spline;
    topology_class = other.topology_class;
    previously_selected_ = other.previously_selected_;
    color_ = other.color_;
  }

  GlobalGuidance::OutputTrajectory &GlobalGuidance::OutputTrajectory::Empty(const Eigen::Vector2d &start, Config *config)
  {
    static std::unique_ptr<OutputTrajectory> empty_trajectory;

    // Make an explicit path in the x direction
    StandaloneGeometricPath path({
        Node(1e5, SpaceTimePoint(start(0), start(1), 0), NodeType::GUARD),
        Node(1e5 + 1, SpaceTimePoint(start(0) + 10., start(1), Config::N / 2), NodeType::GUARD),
        Node(1e5 + 2, SpaceTimePoint(start(0) + 20., start(1), Config::N), NodeType::GUARD),
    });

    empty_trajectory.reset(new OutputTrajectory(path, CubicSpline3D::Empty(start, config)));
    // empty_trajectory.reset(new OutputTrajectory(GeometricPath(), CubicSpline3D::Empty(start, config)));
    return *empty_trajectory;
  }

  GlobalGuidance::~GlobalGuidance()
  {
    /** @todo Fix profiler */
    // RosTools::Instrumentor::Get().EndSession();
  }

  GlobalGuidance::GlobalGuidance() // std::shared_ptr<Config> config)
  {
    PRM_LOG("Initializing Global Guidance");

    // Initialize profiling
    // RosTools::Instrumentor::Get().BeginSession("guidance_planner");

    config_.reset(new Config());
    prm_.Init(config_.get());
    // learning_guidance_.Init(nh_);

    reconfigure_ = std::make_unique<Reconfigure>(config_);

    start_velocity_ = Eigen::Vector2d(0., 0.);
    color_manager_.reset(new ColorManager(config_->n_paths_));

    Reset();
    gsl_set_error_handler(&IntegrationExceptionHandler);
  }

  void GlobalGuidance::LoadObstacles(const std::vector<Obstacle> &obstacles, const std::vector<Halfspace> &static_obstacles)
  {
    obstacles_ = obstacles;
    static_obstacles_ = static_obstacles;
    // if (config_->use_learning)
    //   learning_guidance_.LoadObstacles(obstacles, static_obstacles);
  }

  void GlobalGuidance::LoadStaticObstacles(const std::vector<Halfspace> &static_obstacles)
  {
    static_obstacles_ = static_obstacles;
  }

  void GlobalGuidance::LoadReferencePath(double spline_start, std::unique_ptr<RosTools::Spline2D> &reference_path, double road_width)
  {
    LoadReferencePath(spline_start, reference_path, road_width / 2., road_width / 2.);
  }

  void GlobalGuidance::LoadReferencePath(double spline_start, std::unique_ptr<RosTools::Spline2D> &reference_path,
                                         double road_width_left, double road_width_right)
  {
    PRM_LOG("Global Guidance: Loading Reference Path and Setting Goal Locations");

    ROSTOOLS_ASSERT(!goals_set_, "Please set the goals via SetGoals or LoadReferencePath, but not both!"); // Goals should be set either by SetGoals or by LoadReferencePath, not both!
    goals_set_ = true;

    // DEFINE THE GOALS FOR THE GUIDANCE PLANNER
    int grid_long = config_->longitudinal_goals_;
    int grid_vert = config_->vertical_goals_;

    ROSTOOLS_ASSERT((grid_vert % 2) == 1, "Number of vertical grid points should be odd!");

    // ROSTOOLS_ASSERT(grid_long >= 3, "There should be at least three longitudinal goals (start, end, past end)");
    int vert_start = std::floor((double)grid_vert / 2.);

    double s_start = spline_start;
    double s_best = s_start + Config::DT * (double)Config::N * config_->reference_velocity_;

    double s_step = grid_long > 2 ? (s_best - s_start) / ((double)grid_long - 2.) : s_best - s_start; // -1 for starting at 0
    ROSTOOLS_ASSERT(s_step > 0.05, "Goals should have some spacing between them (Config::reference_velocity_ should not be zero)");

    double width = road_width_left + road_width_right;
    double offset = -road_width_left + width / 2.;
    double v_step = grid_vert > 1 ? width / ((double)(grid_vert - 1)) : 0.;

    goals_.clear();
    // goal_costs_.clear(); // Better goals have a lower score
    for (int i = 0; i < grid_long; i++)
    {
      // Compute the distance at which our goal is longitudinally
      double s = grid_long > 1 ? s_start + (double)i * s_step : s_best;

      // Compute its cost (integer * 2), minimum at desired velocity
      double long_cost = std::abs((grid_long - 2) - i) * 2.;

      // Compute the normal vector to the reference path
      Eigen::Vector2d line_point = reference_path->getPoint(s);
      Eigen::Vector2d normal = reference_path->getVelocity(s).normalized();
      normal = Eigen::Vector2d(-normal(1), normal(0));

      // Place goals orthogonally to the path
      for (int j = -vert_start; j < -vert_start + grid_vert; j++)
      {
        double lat_cost = std::abs(j) * 1.; // Higher cost, the further away from the center line
        double cost = long_cost + lat_cost;
        goals_.emplace_back(line_point + normal * (-offset + ((double)j) * v_step), cost); // Add the goal
      }
    }
  }

  void GlobalGuidance::SetGoals(const std::vector<Goal> &goals)
  {
    ROSTOOLS_ASSERT(!goals_set_, "Please set the goals via SetGoals or LoadReferencePath, but not both!"); // Goals should be set either by SetGoals or by LoadReferencePath, not both!
    goals_set_ = true;
    goals_ = goals;
  }

  void GlobalGuidance::SetStart(const Eigen::Vector2d &start, const double orientation, const double velocity)
  {
    start_ = start;
    orientation_ = orientation;
    start_velocity_ = Eigen::Vector2d(velocity * std::cos(orientation), velocity * std::sin(orientation));

    // if (config_->use_learning)
    //   learning_guidance_.SetStart(start, orientation, velocity);
  }

  bool GlobalGuidance::Update()
  {
    try
    {
      PROFILE_SCOPE("GlobalGuidance::Update");
      PRM_LOG("GlobalGuidance::Update")
      auto &guidance_benchmarker = BENCHMARKERS.getBenchmarker("Guidance Planner");
      auto &prm_benchmarker = BENCHMARKERS.getBenchmarker("PRM");

      guidance_benchmarker.start();

      paths_.clear();
      splines_.clear();

      outputs_.clear();
      learning_outputs_.clear();
      heuristic_outputs_.clear();

      no_message_sent_yet_ = true;
      goals_set_ = false;

      /* Verify validity of input data */
      for (auto &obstacle : obstacles_) // Dynamic obstacles
        ROSTOOLS_ASSERT((int)obstacle.positions_.size() >= Config::N + 1, "Obstacles should have their predictions populated from 0-N");

      PRM_LOG("======== Visibility-PRM ==========");

      prm_benchmarker.start();
      prm_.LoadData(obstacles_, static_obstacles_, start_, orientation_, start_velocity_, goals_);
      Graph &graph = prm_.Update(); // Construct a graph using visibility PRM
      prm_benchmarker.stop();

      PRM_LOG("======== Depth First Path Search ==========");
      {
        PROFILE_SCOPE("Path Search");

        std::vector<std::vector<GeometricPath>> cur_paths; // Collect each set of paths (per goal) here MAKE PREALLOCATED!
        cur_paths.resize(graph.goal_nodes_.size());

        // Search for n_paths to each goal
#pragma omp parallel for num_threads(8)
        for (size_t g = 0; g < graph.goal_nodes_.size(); g++)
        {
          std::vector<Node *> L = {graph.start_node_};

          graph_search_.Search(graph, config_->n_paths_, L, cur_paths[g], graph.goal_nodes_[g]); // Find paths via a graph-search
        }

        // Join all paths
        for (auto &cur_path : cur_paths)
        {
          for (auto &path : cur_path)
            paths_.emplace_back(path);
        }

        // If there are no paths - WARN
        if (paths_.size() == 0 && config_->n_paths_ != 0)
          PRM_WARN("Guidance failed to find a path from the robot position to the goal (using last path)");
      }

      PRM_LOG("======== Filter And Select ==========");
      {
        PROFILE_SCOPE("Path Filtering");

        /* Sort the paths on performance */
        std::sort(paths_.begin(), paths_.end(),
                  [&](const GeometricPath &a, const GeometricPath &b)
                  { return PathSelectionCost(a) < PathSelectionCost(b); });

        // Retrieve n_paths_ topology distinct paths from the sorted paths_
        KeepTopologyDistinctPaths(paths_);

        // Print all paths
        PRM_LOG("Paths:");
        for (auto &path : paths_)
          PRM_LOG("\t" << path << "\b");

        /** Propagate nodes in the graph to the next iteration */
        prm_.PropagateGraph(paths_);
      }

      if (paths_.size() == 0) // Stop here if no paths were found
      {
        guidance_benchmarker.stop();

        // There are no outputs
        outputs_.clear();
        previous_outputs_.clear();
        selected_id_ = -1;

        return false;
      }

      // For each path fit a spline
      PRM_LOG("======== Cubic Splines ==========");
      {
        PROFILE_SCOPE("Cubic Splines");

        for (size_t i = 0; i < paths_.size(); i++)
        {
          auto &path = paths_[i];
          splines_.emplace_back(path, config_.get(), start_velocity_); // Fit Cubic-Splines for each path
          if (config_->optimize_splines_)
            splines_.back().Optimize(obstacles_);
        }
      }

      PRM_LOG("======== Identify ==========");
      {
        PROFILE_SCOPE("Identify");
        outputs_.clear();
        for (size_t i = 0; i < paths_.size(); i++)
          outputs_.emplace_back(paths_[i], splines_[i]);

        color_manager_->Reset(config_->n_paths_);
        IdentifyPreviousHomologies(outputs_); // Find out which of the previous homology classes were preserved
      }

      PRM_LOG("======== Output Selection ==========");
      {
        heuristic_outputs_ = outputs_;
        learning_outputs_ = outputs_;

        // Order the outputs based on some decision making procedure
        // if (!config_->use_learning)
        //   OrderOutputByHeuristic(heuristic_outputs_);
        // else
        //   OrderOutputByLearning(learning_outputs_);

        // outputs_ = config_->use_learning ? learning_outputs_ : heuristic_outputs_;

        // By default the output is now the first element of outputs_
        auto &best_output = outputs_[0];
        selected_id_ = 0;
        if (best_output.previously_selected_)
        {
          PRM_LOG("Selected the same trajectory as last iteration with ID: " << outputs_[selected_id_].topology_class);
        }
        else
        {
          PRM_LOG("Selected a new trajectory with ID: " << outputs_[selected_id_].topology_class);
        }

        previous_outputs_.clear();
        if (config_->track_selected_homology_only_) // Cheaper and should be used when using the first output always
        {
          previous_outputs_.emplace_back(outputs_[0]);
          previous_outputs_[0].previously_selected_ = true;
        }
        else
        {
          for (size_t o = 0; o < outputs_.size(); o++)
          {
            previous_outputs_.emplace_back(outputs_[o]);
            previous_outputs_.back().previously_selected_ = false; // None were selected
          }
          previous_outputs_[0].previously_selected_ = true; // The first output was selected internally
        }
      }

      guidance_benchmarker.stop();
      PRM_LOG("=========== Done ============");

      return true; /* Succesful running */
    }
    catch (IntegrationException &ie)
    {
      LOG_WARN("Integration exception called");
      splines_.clear();
      paths_.clear();
      BENCHMARKERS.getBenchmarker("Guidance Planner").stop();
      return false;
    }
  }

  void GlobalGuidance::IdentifyPreviousHomologies(std::vector<GlobalGuidance::OutputTrajectory> &outputs)
  {
    /** @note Find the output that is homologous with the previously selected trajectory */

    // Set up the new IDs
    IDAssigner id_assigner_(config_->n_paths_);
    for (auto &previous_output : previous_outputs_)
      id_assigner_.MarkIDAsUsed(previous_output.topology_class); // Mark all previous IDs as used

    // Find previously existing topology classes in the new outputs
    int previous_outputs_identified = 0;
    for (auto &output : outputs)
    {
      output.is_new_topology_ = true;

      if (previous_outputs_identified < (int)previous_outputs_.size()) // If not all previous outputs are identified yet
      {
        for (size_t i = 0; i < previous_outputs_.size(); i++) // Go through the previous outputs
        {
          auto &previous_output = previous_outputs_[i];
          if (prm_.AreHomotopicEquivalent(output.path, previous_output.path))
          {
            if (previous_output.previously_selected_)
              output.previously_selected_ = true; // Mark the selected output
            else
              output.previously_selected_ = false;

            output.is_new_topology_ = false;

            output.topology_class = previous_output.topology_class; // Copy the topology class

            bool color_available = color_manager_->ClaimColor(previous_output.color_);
            if (color_available) // It can happen that a previous homotopy (w.r.t old obstacles) results in more than one new homotopy
              output.color_ = previous_output.color_;
            else
              output.color_ = color_manager_->GetColor();

            PRM_LOG("Found existing topology class (ID = " << output.topology_class << ", S = " << output.previously_selected_ << ")");
            previous_outputs_identified++;
            break;
          }
        }
      }

      if (output.is_new_topology_)
      {
        output.topology_class = id_assigner_.GetID();
        PRM_LOG("Found a new topology class (ID = " << output.topology_class << ")");
      }
    }

    for (auto &output : outputs)
    {
      if (output.is_new_topology_)
        output.color_ = color_manager_->GetColor();
    }
  }

  double GlobalGuidance::PathSelectionCost(const GeometricPath &path)
  {
    return 1000. * Goal::FindGoalWithNode(*prm_.GetGoals(), path.nodes_.back()).cost - path.Length3D();
    // return 1000 * path.nodes_.back()->point_.Pos()(0) - path.Length3D();
  }

  void GlobalGuidance::OrderOutputByHeuristic(std::vector<OutputTrajectory> &outputs)
  {
    if (outputs.size() == 0)
      return;

    PRM_LOG("OrderOutputByHeuristic");

    // Select the most suitable guidance trajectory
    std::vector<double> heuristics;
    for (auto &output : outputs)
    {
      // Add costs for all splines
      double heuristic = 0.;

      double goal_cost = Goal::FindGoalWithNode(*prm_.GetGoals(), output.path.path.nodes_.back()).cost;
      heuristic += goal_cost * config_->selection_weight_length_;

      heuristic += output.spline.WeightVelocity() * config_->selection_weight_velocity_;
      heuristic += output.spline.WeightAcceleration() * config_->selection_weight_acceleration_;

      if (output.previously_selected_)
        heuristic *= 1. / config_->selection_weight_consistency_; // Prefer the previously selected output
      heuristics.push_back(heuristic);
    }

    // Sort splines by the heuristic
    std::vector<int> indices(splines_.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&](const int a, const int b)
              { return heuristics[a] < heuristics[b]; });
    sorted_indices_ = indices;

    // Apply the sorting to the splines vector
    std::vector<OutputTrajectory> ordered_outputs;
    for (size_t i = 0; i < outputs.size(); i++)
      ordered_outputs.emplace_back(outputs[indices[i]]);
    outputs = ordered_outputs;
  }

  void GlobalGuidance::OrderOutputByLearning(std::vector<OutputTrajectory> &outputs)
  {
    (void)outputs;
    // if (outputs.size() <= 1)
    //   return;

    // PRM_LOG("OrderOutputByLearning");

    // guidance_planner::select_guidance srv;
    // for (size_t i = 0; i < outputs.size(); i++)
    // {
    //   guidance_planner::RightAvoidanceMSG h_signature_msg;
    //   GuidancePlanner::Goal main_goal = *std::min_element(goals_.begin(), goals_.end(),
    //                                                       [&](const GuidancePlanner::Goal &a, const GuidancePlanner::Goal &b)
    //                                                       { return a.cost < b.cost; });
    //   if (goals_.front().cost <= main_goal.cost)
    //     main_goal = goals_.front();

    //   std::vector<bool> right = PassesRight(i, main_goal.pos);
    //   for (size_t i_obs = 0; i_obs < right.size(); i_obs++)
    //     h_signature_msg.right_avoidance.push_back((double)right[i_obs]);

    //   srv.request.h_signatures.push_back(h_signature_msg);
    // }

    // // The trained timestep is 0.4
    // auto &poses_list = learning_guidance_.GetPosesList();
    // double full_diff = (poses_list.back().timestamp - poses_list.front().timestamp).toSec();
    // double time_diff = full_diff - (std::fmod(full_diff, 0.4));
    // for (size_t i_poses = 0; i_poses < poses_list.size(); i_poses++)
    // {
    //   if ((poses_list.back().timestamp - poses_list[i_poses].timestamp).toSec() <= time_diff)
    //   {
    //     srv.request.robot_trajectory.x.push_back(poses_list[i_poses].position.x());
    //     srv.request.robot_trajectory.y.push_back(poses_list[i_poses].position.y());
    //     // ROS_INFO_STREAM("time diff: " << time_diff << " -> " << (poses_list.back().timestamp - poses_list[i_poses].timestamp).toSec());
    //     time_diff -= 0.4;
    //   }
    // }

    // for (auto &obs_saved : learning_guidance_.GetPreviousObstacles())
    // {
    //   guidance_planner::ObstacleMSG obs_aux;
    //   obs_aux.id = obs_saved.id_;
    //   obs_aux.radius = obs_saved.radius_;
    //   full_diff = (obs_saved.positions_.back().timestamp - obs_saved.positions_.front().timestamp).toSec();
    //   time_diff = full_diff - (std::fmod(full_diff, 0.4));

    //   for (size_t i_poses = 0; i_poses < obs_saved.positions_.size(); i_poses++)
    //   {

    //     if ((obs_saved.positions_.back().timestamp - obs_saved.positions_[i_poses].timestamp).toSec() <= time_diff)
    //     {
    //       obs_aux.pos_x.push_back(obs_saved.positions_[i_poses].position.x());
    //       obs_aux.pos_y.push_back(obs_saved.positions_[i_poses].position.y());
    //       time_diff -= 0.4;
    //     }
    //   }

    //   srv.request.previous_obstacles.push_back(obs_aux);
    // }

    // if (learning_guidance_.CallServer(srv))
    // {
    //   // Sort splines by the heuristic
    //   std::vector<int> indices(splines_.size());
    //   // ROS_INFO_STREAM(previous_outputs_[0].path);

    //   for (size_t i_output = 0; i_output < outputs.size(); i_output++)
    //   {
    //     if (outputs[i_output].previously_selected_)
    //       srv.response.cost_guidances[i_output] *= 1. / config_->selection_weight_consistency_;
    //   }

    //   std::iota(indices.begin(), indices.end(), 0);
    //   std::sort(indices.begin(), indices.end(), [&](const int a, const int b)
    //             { return srv.response.cost_guidances[a] < srv.response.cost_guidances[b]; });
    //   sorted_indices_ = indices;

    //   // Apply the sorting to the splines vector
    //   std::vector<OutputTrajectory> ordered_outputs;
    //   for (size_t i = 0; i < outputs.size(); i++)
    //     ordered_outputs.push_back(outputs[indices[i]]);
    //   outputs = ordered_outputs;
    // }
    // else
    // {
    //   ROS_ERROR_ONCE("Failed to call service select_guidance");
    //   OrderOutputByHeuristic(outputs);
    // }
  }

  void GlobalGuidance::KeepTopologyDistinctPaths(std::vector<GeometricPath> &paths)
  {
    PROFILE_FUNCTION();

    if (paths.size() == 0)
      return;

    /** @note paths are sorted, so that the first occurence in the list is the best path and all equivalent further paths can be removed in favor of the first */

    std::vector<GeometricPath> topology_distinct_paths; // Construct a new list with topology distinct paths
    topology_distinct_paths.emplace_back(paths.front());

    for (size_t i = 1; (i < paths.size()) && ((int)topology_distinct_paths.size() < config_->n_paths_); i++) // Up until n_paths_ are ready
    {
      auto &candidate_path = paths[i];

      bool distinct = true;
      for (auto &path : topology_distinct_paths) // If any of the paths in the list is equivalent with this one, it is not distinct
      {
        if (prm_.AreHomotopicEquivalent(path, candidate_path))
        {
          distinct = false;
          break;
        }
      }

      if (distinct)
        topology_distinct_paths.emplace_back(paths[i]); // Add a path if it is distinct from the topologies that we know
    }

    paths = topology_distinct_paths;
  }

  void GlobalGuidance::Reset()
  {
    PRM_LOG("Reset()");

    // Forget outputs
    paths_.clear();
    splines_.clear();
    outputs_.clear();
    previous_outputs_.clear();

    // Forget the graph
    prm_.Reset();

    for (auto &obstacle : obstacles_) // Ensure that the obstacles have long enough predictions
    {
      obstacle.positions_.resize(Config::N + 1);
      for (int k = 0; k <= Config::N; k++)
        obstacle.positions_[k] = Eigen::Vector2d(100., 100.);

      obstacle.radius_ = 0.;
    }
  }

  GlobalGuidance::OutputTrajectory &GlobalGuidance::GetGuidanceTrajectory(int trajectory_id)
  {
    if (trajectory_id >= (int)outputs_.size())
    {
      LOG_WARN("Trying to retrieve a trajectory that does not exist!");

      if (no_message_sent_yet_ && outputs_.size() == 0)
      {
        // ROS_WARN("The guidance planner could not find a path");
        no_message_sent_yet_ = false;
      }

      return GlobalGuidance::OutputTrajectory::Empty(start_, config_.get());
    }

    return outputs_[trajectory_id]; // Return the guidance trajectory
  }

  std::vector<bool> GlobalGuidance::PassesRight(int output_id, const Eigen::Vector2d &goal)
  {
    if (output_id >= (int)outputs_.size())
    {
      LOG_WARN("Trying to get the cost of a trajectory that does not exist!");
      std::vector<bool> empty;
      return empty;
    }
    GuidancePlanner::GeometricPath h_def_path = outputs_[output_id].path;
    GuidancePlanner::SpaceTimePoint goal_point(goal.x(), goal.y(), h_def_path.nodes_.back()->point_.Time());
    GuidancePlanner::Node goal_node(h_def_path.nodes_.back()->id_, goal_point, h_def_path.nodes_.back()->type_);
    h_def_path.nodes_.push_back(&goal_node);
    return prm_.PassesRight(h_def_path); // Return the guidance trajectory
  }

  double GlobalGuidance::GetHomotopicCost(int output_id, const GeometricPath &path)
  {
    if (output_id >= (int)outputs_.size())
    {
      LOG_WARN("Trying to get the cost of a path that does not exist!");
      return -1;
    }

    return this->prm_.GetHomotopicCost(outputs_[output_id].path, path); // Return the guidance trajectory
  }

  void GlobalGuidance::OverrideSelectedTrajectory(int topology_class, bool set_none)
  {
    if (selected_id_ != -1)
      previous_outputs_[selected_id_].previously_selected_ = false; // Undo the last selection

    if (!set_none)
    {
      int output_id = -1; // Find the topology class
      for (size_t i = 0; i < outputs_.size(); i++)
      {
        if (outputs_[i].topology_class == topology_class)
        {
          output_id = i;
          break;
        }
      }

      ROSTOOLS_ASSERT(output_id != -1, "Trying to select a topology class that does not exist");

      previous_outputs_[output_id].previously_selected_ = true; // But the given one was
      selected_id_ = output_id;
    }
    else
    {
      selected_id_ = -1;
    }
  }

  int GlobalGuidance::NumberOfGuidanceTrajectories() const { return (int)(outputs_.size()); }

  int GlobalGuidance::GetIdSamePath(const GeometricPath &path) // Not used!
  {
    for (size_t return_id = 0; return_id < this->paths_.size(); return_id++)
    {
      if (prm_.AreHomotopicEquivalent(path, paths_[return_id]))
      {
        return return_id;
      }
    }
    return -1;
  }

  void GlobalGuidance::Visualize(bool highlight_selected, int only_path_nr)
  {
    PROFILE_SCOPE("GlobalGuidance::Visualize");
    PRM_LOG("======== Visualization ==========");
    VisualizeObstacles();
    prm_.Visualize();
    VisualizeGeometricPaths(only_path_nr);
    VisualizeTrajectories(highlight_selected, only_path_nr);

    if (config_->debug_output_)
    {
      VisualizeSplinePoints();
      VisualizeDebug();
    }
    PRM_LOG("=================================");
  }

  /** @brief Helper function*/
  void GlobalGuidance::VisualizePath(RosTools::ROSLine &line, GeometricPath &path)
  {
    Node *prev_node;
    bool first_node = true;
    for (auto &node : path.nodes_)
    {
      if (!first_node)
        line.addLine(node->point_.MapToTime(), prev_node->point_.MapToTime());
      else
        first_node = false;

      prev_node = node;
    }
  }

  void GlobalGuidance::VisualizeGeometricPaths(int path_nr)
  {
    auto &path_visuals = VISUALS.getPublisher("guidance_planner/geometric_paths");

    RosTools::ROSLine &path_line = path_visuals.getNewLine();
    path_line.setScale(0.15, 0.15, 0.15);

    // Visualize the path for each output
    for (size_t i = 0; i < outputs_.size(); i++)
    {
      auto &path = outputs_[i].path;

      if (path_nr != -1 && path_nr != (int)i)
        continue;

      /*if (previous_outputs_[i].previously_selected_)
        path_line.setColorInt(4, 1., RosTools::Colormap::BRUNO);
      else*/
      path_line.setColorInt(outputs_[i].color_, config_->n_paths_, 0.75);

      VisualizePath(path_line, path.path);
    }

    path_visuals.publish();
  }

  void GlobalGuidance::VisualizeSplinePoints()
  {
    // CONTROL POINTS OF CUBIC SPLINES
    for (auto &spline : splines_)
      spline.Visualize();
  }

  void GlobalGuidance::VisualizeObstacles()
  {
    auto &obstacle_visuals = VISUALS.getPublisher("guidance_planner/obstacles_3d");
    auto &disc = obstacle_visuals.getNewPointMarker("CYLINDER");

    // Visualize the obstacles
    int j = 0;
    for (auto &obstacle : obstacles_)
    {
      disc.setScale(obstacle.radius_ * 2., obstacle.radius_ * 2., Config::DT);
      for (int k = 0; k < Config::N; k++)
      {
        // Transparent
        disc.setColorInt(obstacle.id_, (1. - config_->visuals_transparency_) * std::pow(((double)(Config::N - k)) / (double)Config::N, 2.), RosTools::Colormap::BRUNO);

        disc.addPointMarker(Eigen::Vector3d(obstacle.positions_[k](0), obstacle.positions_[k](1), (float)k * Config::DT));
      }

      j++;
    }
    obstacle_visuals.publish();
  }

  void GlobalGuidance::VisualizeTrajectories(bool highlight_selected, int path_nr)
  {
    auto &trajectory_visuals = VISUALS.getPublisher("guidance_planner/trajectories");

    RosTools::ROSLine &line = trajectory_visuals.getNewLine();
    line.setScale(0.15, 0.15);

    RosTools::ROSTextMarker text_marker = trajectory_visuals.getNewTextMarker();
    text_marker.setScale(1.0);

    for (size_t i = 0; i < outputs_.size(); i++)
    {
      auto &spline = outputs_[i].spline;

      if (path_nr != -1 && path_nr != (int)i)
        continue;

      if (highlight_selected && previous_outputs_[i].previously_selected_)
      {
        // line.setScale(0.3, 0.3);

        if (config_->use_learning) // Blue
        {
          text_marker.setColorInt(3.0, 1.0, RosTools::Colormap::BRUNO);
          line.setColorInt(3.0, 1.0, RosTools::Colormap::BRUNO);
        }
        else // Red
        {
          text_marker.setColor(131. / 255., 10. / 255., 72. / 255., 1.0);
          line.setColor(131. / 255., 10. / 255., 72. / 255., 1.0);
        }
      }
      else // Color scale
      {
        line.setScale(0.15, 0.15);
        line.setColorInt(outputs_[i].color_, config_->n_paths_, 0.75);
      }

      std::vector<Eigen::Vector3d> &points = spline.GetSamples(); // Get samples on the current spline

      if (config_->show_trajectory_indices_)
      {
        text_marker.setText(std::to_string(outputs_[i].topology_class)); // Add the spline number
        text_marker.setColorInt(outputs_[i].color_, config_->n_paths_, 1.);
      }

      // Draw a line for the time scaled points
      for (size_t j = 0; j < points.size(); j++)
      {
        Eigen::Vector3d &cur_vec = points[j];

        if (j > 0)
        {
          Eigen::Vector3d &prev_vec = points[j - 1];
          line.addLine(prev_vec, cur_vec);
        }
      }

      if (config_->show_trajectory_indices_)
        text_marker.addPointMarker(points.back() + Eigen::Vector3d(0., 0., 3.0));
    }
    trajectory_visuals.publish();
  }

  void GlobalGuidance::VisualizeDebug()
  {
    if (splines_.size() != 0)
    {
      auto &spline_visuals = VISUALS.getPublisher("guidance_planner/spline");
      // VISUALIZE THE USED PART OF THE SPLINE //
      RosTools::ROSLine &guidance_path = spline_visuals.getNewLine();

      if (splines_.size() > 0)
      {
        guidance_path.setScale(0.15, 0.15);

        int n_segments = 20;
        double step = 0.1;
        int offset = 0; // The first segment is indexed as such

        RosTools::Spline2D &selected_spline = splines_[0].GetPath();

        // Go through all used segments and plot them
        for (int i = offset; i < std::min(offset + n_segments, selected_spline.numSegments()); i++)
        {
          double start_segment = selected_spline.getSegmentStart(i);
          double end_segment = selected_spline.getSegmentStart(i + 1);

          guidance_path.setColorInt(i, 10, 0.7);

          for (double t = start_segment + step; t <= end_segment; t += step)
          {
            guidance_path.addLine(selected_spline.getPoint(t), selected_spline.getPoint(t - step), 0.1);
          }
        }
      }
      spline_visuals.publish();
    }
  }

  void GlobalGuidance::ExportData(RosTools::DataSaver &data_saver) // Export data for analysis
  {
    data_saver.AddData("n_paths", paths_.size());
  }

  double GlobalGuidance::GetLastRuntime()
  {
    return BENCHMARKERS.getBenchmarker("Guidance Planner").getLast();
  }

} // namespace GuidancePlanner
