#include "guidance_planner/global_guidance.h"

namespace GuidancePlanner
{
  GlobalGuidance::~GlobalGuidance()
  {
    // TODO
    //  RosTools::Instrumentor::Get().EndSession();
  }

  GlobalGuidance::GlobalGuidance()
  {
    PRM_LOG("Initializing Global Guidance");

    // Initialize profiling
    // RosTools::Instrumentor::Get().BeginSession("Guidance Planner");

    config_.reset(new Config());
    prm_.Init(nh_, config_.get());

    first_reconfigure_callback_ = true;
    ros::NodeHandle nh_guidance("guidance_planner");
    reconfigure_server_.reset(new dynamic_reconfigure::Server<GuidancePlanner::GuidancePlannerConfig>(reconfig_mutex_, nh_guidance));
    reconfigure_server_->setCallback(boost::bind(&GlobalGuidance::ReconfigureCallback, this, _1, _2));

    /* Initialize visuals */
    ros_visuals_.reset(new RosTools::ROSMarkerPublisher(nh_, "lmpcc/homotopy/guidance_trajectories", "map", 500));
    ros_bspline_visuals_.reset(new RosTools::ROSMarkerPublisher(nh_, "lmpcc/homotopy/spline_points", "map", 200));
    ros_selected_visuals_.reset(new RosTools::ROSMarkerPublisher(nh_, "lmpcc/homotopy/selected_guidance", "map", 200));
    ros_guidance_path_visuals_.reset(new RosTools::ROSMarkerPublisher(nh_, "lmpcc/homotopy/guidance_path", "map", 200));
    ros_obstacle_visuals_.reset(new RosTools::ROSMarkerPublisher(nh_, "lmpcc/homotopy/obstacles_3d", "map", 200));
    ros_path_visuals_.reset(new RosTools::ROSMarkerPublisher(nh_, "lmpcc/homotopy/geometric_paths", "map", 200));

    /* Initialize benchmarkers for debugging purposes */
    benchmarkers_.push_back(std::unique_ptr<RosTools::Benchmarker>(new RosTools::Benchmarker("Guidance Planner", false, 0)));
    benchmarkers_.push_back(std::unique_ptr<RosTools::Benchmarker>(new RosTools::Benchmarker("Visibility-PRM", false, 0)));
    benchmarkers_.push_back(std::unique_ptr<RosTools::Benchmarker>(new RosTools::Benchmarker("Path Search", false, 0)));

    /* Initialize service client */
    this->estimate_cost_client = nh_.serviceClient<guidance_planner::guidances_estimate_cost>("guidances_estimate_cost");

    // thread_pool_.reset(new ThreadPool(1));

    start_velocity_ = Eigen::Vector2d(0., 0.);

    Reset();
  }

  void GlobalGuidance::UpdateLearning(){
    this->learning_selected_id = 0;
    double minimum_achieved_cost = 9999.9;
    for (int i = 0; i < this->NumberOfGuidanceTrajectories(); i++){
        // ROS_INFO_STREAM("H-cost: " << res.costs.back());
        guidance_planner::LeftHMSG h_signature_msg;
        Eigen::Vector2d main_goal = goals_[0].pos;
        std::vector<bool> right = this->LeftPassingH(i, main_goal);
        for(int i_obs = 0; i_obs < (int)right.size(); i_obs++){
            h_signature_msg.left_passing.push_back((double)right[i_obs]);
        }
        guidance_planner::guidances_estimate_cost srv;
        // INCLUDE ROBOT TRAJECTORY
        // The trained timestep is 0.4
        double full_diff = (poses_list.back().timestamp - poses_list.front().timestamp).toSec();
        double time_diff = full_diff - (std::fmod(full_diff, 0.4));
        for (int i_poses = 0; i_poses < poses_list.size(); i_poses++){
          if ((poses_list.back().timestamp - poses_list[i_poses].timestamp).toSec() <= time_diff){
            srv.request.robot_trajectory.x.push_back(poses_list[i_poses].position.x());
            srv.request.robot_trajectory.y.push_back(poses_list[i_poses].position.y());
            time_diff -= 0.4;
            ROS_INFO_STREAM("ADDED POSE " << i_poses);
          }
        }
        for (auto& obs_saved :previous_obstacles_list){
          guidance_planner::ObstacleMSG obs_aux;
          obs_aux.id = obs_saved.id_;
          obs_aux.radius = obs_saved.radius_;
          for (int i_poses = 0; i_poses < obs_saved.positions_.size(); i_poses++){
            if ((obs_saved.positions_.back().timestamp - obs_saved.positions_[i_poses].timestamp).toSec() <= time_diff){
              obs_aux.pos_x.push_back(obs_saved.positions_[i_poses].position.x());
              obs_aux.pos_y.push_back(obs_saved.positions_[i_poses].position.y());
              time_diff -= 0.4;
              ROS_INFO_STREAM("ADDED POSE " << i_poses);
            }
          }
          srv.request.previous_obstacles.push_back(obs_aux);
        }
        srv.request.h_signature = h_signature_msg;
        if (this->estimate_cost_client.call(srv))
        {
          if (srv.response.cost < minimum_achieved_cost){
              this->learning_selected_id = i;
              minimum_achieved_cost = srv.response.cost;
          }
        }
        else
        {
          ROS_ERROR("Failed to call service guidances_estimate_cost");
        }
    }
  }

  void GlobalGuidance::LoadObstacles(const std::vector<Obstacle> &obstacles, const std::vector<RosTools::Halfspace> &static_obstacles)
  {
    obstacles_ = obstacles;
    static_obstacles_ = static_obstacles;
    if (config_->use_learning){
      ros::Time timestamp = ros::Time::now();
      // Iterate over the current obstacles
      for (const auto& currentObstacle : obstacles) {
          // Check if the obstacle exists in the original vector
          auto it = std::find_if(previous_obstacles_list.begin(), previous_obstacles_list.end(),
              [currentId = currentObstacle.id_](const ObstacleInfo& o) {
                  return o.id_ == currentId;
              });

          // If the obstacle doesn't exist in the original vector, add it
          if (it == previous_obstacles_list.end()) {
              ObstacleInfo obs_new;
              obs_new.id_ = currentObstacle.id_;
              obs_new.radius_ = currentObstacle.radius_;
              PositionTime pos_new;
              pos_new.timestamp = timestamp;
              pos_new.position = currentObstacle.positions_[0];
              obs_new.positions_.push_back(pos_new);
              previous_obstacles_list.push_back(obs_new);
          }
          // Update the positions of the existing obstacle
          else{
              // Check time interval and how many to store
              if (it->positions_.size() > 0 && (it->positions_.front().timestamp - timestamp >= ros::Duration(2.8))) {
                // Erase the first (oldest) position
                it->positions_.erase(it->positions_.begin());
              }
              PositionTime pos_new;
              pos_new.timestamp = timestamp;
              pos_new.position = currentObstacle.positions_[0];
              it->positions_.push_back(pos_new);
          }
      }

      // Iterate over the original obstacles and remove the ones that don't appear in the current vector
      previous_obstacles_list.erase(std::remove_if(previous_obstacles_list.begin(), previous_obstacles_list.end(),
          [&](const ObstacleInfo& o) {
              auto it = std::find_if(obstacles.begin(), obstacles.end(),
                  [currentId = o.id_](const Obstacle& co) {
                      return co.id_ == currentId;
                  });
              return it == obstacles.end();
          }),
          previous_obstacles_list.end());
      }
  }

  void GlobalGuidance::LoadReferencePath(double spline_start, RosTools::CubicSpline2D<tk::spline> *reference_path, double road_width)
  {
    LoadReferencePath(spline_start, reference_path, road_width / 2., road_width / 2.);
  }

  void GlobalGuidance::LoadReferencePath(double spline_start, RosTools::CubicSpline2D<tk::spline> *reference_path,
                                         double road_width_left, double road_width_right)
  {
    PRM_LOG("Global Guidance: Loading Reference Path and Setting Goal Locations");

    ROSTOOLS_ASSERT(!goals_set_, "Please set the goals via SetGoals or LoadReferencePath, but not both!"); // Goals should be set either by SetGoals or by LoadReferencePath, not both!
    goals_set_ = true;

    // DEFINE THE GOALS FOR THE GUIDANCE PLANNER
    int grid_long = config_->longitudinal_goals_;
    int grid_vert = config_->vertical_goals_;

    ROSTOOLS_ASSERT((grid_vert % 2) == 1, "Number of vertical grid points should be odd!");
    int vert_start = std::floor((double)grid_vert / 2.);

    double s_start = spline_start;
    double s_best = s_start + Config::DT * (double)Config::N * config_->reference_velocity_;
    double s_step = (s_best - s_start) / ((double)grid_long - 1.); // -1 for starting at 0
    ROSTOOLS_ASSERT(s_step > 0.05, "Goals should have some spacing between them (Config::reference_velocity_ should not be zero)");

    double width = road_width_left + road_width_right;
    double offset = -road_width_left + width / 2.;
    double v_step = width / ((double)(grid_vert - 1));

    goals_.clear();
    // goal_costs_.clear(); // Better goals have a lower score
    for (int i = 0; i < grid_long; i++)
    {
      // Compute the distance at which our goal is longitudinally
      double s = s_start + (double)i * s_step;

      // Compute its cost (integer * 2), minimum at desired velocity
      double long_cost = std::abs((grid_long - 1) - i) * 2.;

      // Compute the normal vector to the reference path
      Eigen::Vector2d line_point = reference_path->GetPoint(s);
      Eigen::Vector2d normal = reference_path->GetVelocity(s).normalized();
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
    if (this->config_->use_learning){
      ros::Time timestamp = ros::Time::now();
      if (this->poses_list.size() > 0 && (poses_list.front().timestamp - timestamp >= ros::Duration(2.8))) {
        // Erase the first (oldest) position
        poses_list.erase(poses_list.begin());
      }
      PoseInfo pose_new;
      pose_new.orientation = orientation;
      pose_new.position = start;
      pose_new.timestamp = timestamp;
      pose_new.velocity = velocity;
      poses_list.push_back(pose_new);
    }
  }

  bool GlobalGuidance::Update()
  {
    PROFILE_SCOPE("GlobalGuidance::Update");
    PRM_LOG("GlobalGuidance::Update")
    benchmarkers_[0]->start();

    no_message_sent_yet_ = true;
    goals_set_ = false;

    /* Verify validity of input data */
    for (auto &obstacle : obstacles_) // Dynamic obstacles
      ROSTOOLS_ASSERT((int)obstacle.positions_.size() >= Config::N + 1, "Obstacles should have their predictions populated from 0-N");

    PRM_LOG("======== PRM ==========");

    prm_.LoadData(obstacles_, static_obstacles_, start_, orientation_, start_velocity_, goals_, selected_id_);
    benchmarkers_[1]->start();
    Graph &graph = prm_.Update(); // Construct a graph using visibility PRM
    benchmarkers_[1]->stop();

    PRM_LOG("======== Path Search ==========");
    {
      PROFILE_SCOPE("Path Search");
      benchmarkers_[2]->start();

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
      paths_.clear();
      for (auto &cur_path : cur_paths)
      {
        for (auto &path : cur_path)
          paths_.emplace_back(path);
      }

      FilterPaths();                    // Remove any poorly formed paths
      RemoveHomotopicEquivalentPaths(); // Remove paths in duplicate topologies

      /* Select the best paths*/
      std::sort(paths_.begin(), paths_.end(),
                [&](const GeometricPath &a, const GeometricPath &b)
                { return PathSelectionCost(a) > PathSelectionCost(b); });

      paths_.resize(std::min((int)paths_.size(), config_->n_paths_)); // Keep the best num_paths paths

      // If there are no paths - WARN
      if (paths_.size() == 0 && config_->n_paths_ != 0)
        PRM_WARN("Guidance failed to find a path from the robot position to the goal (using last path)");

      benchmarkers_[2]->stop();
    }

    // Print all paths
    PRM_LOG("Paths:");
    for (auto &path : paths_)
      PRM_LOG("\t" << path << "\b");

    PRM_LOG("======== Association ==========");
    {
      PROFILE_SCOPE("Association");
      // Association Step 1: Check the segment association IDs of the nodes in each path and assign an ID to each Path based
      // on it While each node connects only to two guards, a node can appear in more than one path, due to the path search.
      // A path is therefore associated with a sequence of nodes.
      // Node IDs are identified by segment_association_id_

      // Create a list of indices to be used for the paths
      /** @NOTE: In some cases the previous path ID may be higher than the number of paths we have currently, that is okay */
      std::vector<int> available_ids(config_->n_paths_);
      std::iota(available_ids.begin(), available_ids.end(), 0);

      AssignIDsToKnownPaths(available_ids); // Consistently assign IDs to paths that we found in the previous iteration
      AssignIDsToNewPaths(available_ids);   // Now for all paths that do not have an ID assigned, assign them in order
      OrderPaths();                         // Try to keep the same guidance trajectories in the same place for consistency

      // Association Step 2: Save the known paths via an ID and a list of segment IDs
      known_paths_.clear();
      for (auto &path : paths_)
      {
        known_paths_.push_back(path.association_); // Save the assocation of each path (we need it in the next iteration)
        PRM_LOG("Saving path [" << known_paths_.back().id_ << "]: " << path);
      }

      /** Transfer the path association information to the nodes in it AND propagate the nodes to the next iteration */
      prm_.TransferPathInformationAndPropagate(paths_, known_paths_);
    }

    // For each path fit a spline
    PRM_LOG("======== Cubic Splines ==========");
    {
      PROFILE_SCOPE("Cubic Splines");
      if (paths_.size() == 0) // Skip splines if no paths were found, so that we can still use the previous splines
      {
        benchmarkers_[0]->stop();
        return false;
      }

      // Test the passing left feature
      /*for (auto &path : paths_)
      {
        std::vector<bool> pass_left = prm_.GetLeftPassingVector(path);

        std::cout << path.association_.id_ << ": ";
        for (size_t i = 0; i < pass_left.size(); i++)
        {
          if (pass_left[i])
            std::cout << "L | ";
          else
            std::cout << "R | ";
        }
        std::cout << "\b\b\n";
      }*/

      splines_.clear();

      for (auto &path : paths_)
      {
        // Fit Cubic-Splines for each path
        splines_.emplace_back(path, config_.get(), start_velocity_);

        if (config_->optimize_splines_)
          splines_.back().Optimize(obstacles_);
      }

      // Spline selection
      OrderSplinesByHeuristic();
    }
    PRM_LOG("=========================");

    benchmarkers_[0]->stop();

    return true; /* Succesful running */
  }

  double GlobalGuidance::PathSelectionCost(const GeometricPath &path) { return 1000 * path.nodes_.back()->point_.Pos()(0) - path.Length3D(); }

  void GlobalGuidance::AssignIDsToKnownPaths(std::vector<int> &available_ids)
  {
    for (auto &path : paths_)
    {
      ROSTOOLS_ASSERT(path.association_.AllSegmentsAssigned(),
                      "A segment in this path was missing a segment ID"); // By construction all segments should have an ID
      for (auto &previous_path_assocation : known_paths_)
      {
        if (path.association_.Matches(previous_path_assocation))
        {
          PRM_LOG("Path " << path << " matches the previously known path with ID: " << previous_path_assocation.id_);
          path.association_.id_ = previous_path_assocation.id_;
          path_id_was_known_[path.association_.id_] = true; // Ensures that this path was not randomly given the ID - but obtained it

          auto iterator_at_available_id = std::find(available_ids.begin(), available_ids.end(),
                                                    path.association_.id_); // Should be updated after erasing
          ROSTOOLS_ASSERT(iterator_at_available_id != std::end(available_ids),
                          "When assigning path IDs, an ID was assigned twice"); // This ID must still be available

          available_ids.erase(iterator_at_available_id); // The ID is not available anymore

          break;
        }
      }
    }
  }

  void GlobalGuidance::AssignIDsToNewPaths(std::vector<int> &available_ids)
  {
    int assign_increment = 0;

    for (auto &path : paths_)
    {
      // If this path has no id
      if (!path.association_.Assigned())
      {
        // Assign the next id
        path.association_.id_ = available_ids[assign_increment];
        path_id_was_known_[path.association_.id_] = false; // This is a new path ID (no consistency bonus)

        PRM_LOG("Path without associated nodes found, setting ID to " << path.association_.id_);

        assign_increment++;
      }
    }
  }

  void GlobalGuidance::FilterPaths()
  {
    // Deactivated
  }

  void GlobalGuidance::OrderPaths() // Keep paths that previously were there in the same place!
  {
    std::vector<GeometricPath> ordered_paths;
    ordered_paths.resize(paths_.size());

    std::vector<bool> path_assigned(paths_.size(), false);
    std::vector<bool> place_assigned(paths_.size(), false);

    for (size_t i = 0; i < paths_.size(); i++)
    {
      auto &path = paths_[i];
      // NOTE: paths_.size(), because if known_paths_ is larger than paths_ we cannot keep the structure anyways
      for (size_t j = 0; j < std::min(paths_.size(), known_paths_.size()); j++)
      {
        auto &known_path = known_paths_[j];
        if (path.association_.id_ == known_path.id_)
        {

          ordered_paths[j] = path;
          path_assigned[i] = true;
          place_assigned[j] = true;
        }
      }
    }

    for (size_t i = 0; i < paths_.size(); i++)
    {

      if (path_assigned[i])
        continue;

      for (size_t j = 0; j < paths_.size(); j++) // Paths size, because it may need to be larger than the known_paths
      {
        if (!place_assigned[j]) // Find the first free place
        {
          ordered_paths[j] = paths_[i];
          place_assigned[j] = true;
          break;
        }
      }
    }

    paths_ = ordered_paths;
  }

  void GlobalGuidance::RemoveHomotopicEquivalentPaths()
  {
    PROFILE_FUNCTION();
    // 1) Track in how many paths each connector is present so that we may remove connectors that are not in any path in
    // the end
    std::map<Node *, int> connector_count;
    for (auto &path : paths_)
    {
      for (auto &node : path.nodes_)
        connector_count[node]++;
    }

    // Check each pair of paths and remove homotopic equivalent paths from the list (keeping the "best" path)
    std::vector<bool> removal_marker(paths_.size(), false);
    for (size_t i = 0; i < paths_.size(); i++)
    {
      if (removal_marker[i]) // If this one was removed already - skip
        continue;

      for (size_t j = 0; j < paths_.size(); j++) // For all other paths
      {
        if (i == j || removal_marker[j]) // If this one is removed or is the same as the other - skip
          continue;

        if (removal_marker[i]) // break from the "j" loop as path i will be replaced by another
          break;

        // If these two paths are homotopic equivalent (can also be checked by verifying that the associations are the
        // same)
        if (paths_[i].association_.Matches(paths_[j].association_) || prm_.AreHomotopicEquivalent(paths_[i], paths_[j]))
        {
          PRM_LOG(paths_[i] << " and " << paths_[j] << " are homotopic equivalent paths");

          // Keep the "best" path
          if (prm_.FirstPathIsBetter(paths_[i], paths_[j]))
          {
            PRM_LOG("Marked the second path for removal");

            removal_marker[j] = true;
            paths_[i].association_.Merge(paths_[j].association_,
                                         selected_id_); // Merge associations (preferring selected IDs)
          }
          else
          {
            PRM_LOG("Marked the first path for removal");

            removal_marker[i] = true;
            paths_[j].association_.Merge(paths_[i].association_,
                                         selected_id_); // Merge associations (preferring selected IDs)
          }
        }
      }
    }

    // Remove marked elements from the paths vector
    for (int i = (int)paths_.size() - 1; i >= 0; i--)
    {
      if (removal_marker[i])
      {
        for (auto &node : paths_[i].nodes_)
        {
          if (node->type_ == NodeType::CONNECTOR) // Mark all connector nodes in the paths as "removed"
          {
            connector_count[node]--;

            if (connector_count[node] == 0)
            {
              node->replaced_ = true;
              PRM_LOG("Erasing Node " << node->id_);
            }
          }
        }

        PRM_LOG("Erasing " << paths_[i]); // If the node was removed prior to this, then the log may be empty
        paths_.erase(paths_.begin() + i);
      }
    }
  }

  void GlobalGuidance::OrderSplinesByHeuristic()
  {
    if (splines_.size() == 0)
      return;

    // Select the most suitable guidance trajectory
    std::vector<double> spline_costs;
    CubicSpline3D *best_spline = nullptr;

    for (auto &spline : splines_)
    {
      // Standard penalty
      double consistency_weight = 1.;

      /* The last check verifies that the new path did not by accident get assigned the selected ID */
      if (spline.id_ == selected_id_ && path_id_was_known_[spline.id_])
        consistency_weight = 0.;

      // Add costs for all splines
      double spline_cost = 0.;
      spline_cost += spline.WeightPathLength() * config_->selection_weight_length_;
      spline_cost += spline.WeightVelocity() * config_->selection_weight_velocity_;
      spline_cost += spline.WeightAcceleration() * config_->selection_weight_acceleration_;
      spline_cost += consistency_weight * config_->selection_weight_consistency_;
      spline_costs.push_back(spline_cost);
    }

    // Sort splines by their cost
    std::vector<int> indices(splines_.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&](const int a, const int b)
              { return spline_costs[a] < spline_costs[b]; });
    
    this->sorted_indices = indices;

    // Apply the sorting to the splines vector
    std::vector<CubicSpline3D> ordered_splines;
    for (size_t i = 0; i < splines_.size(); i++)
      ordered_splines.push_back(splines_[indices[i]]);
    splines_ = ordered_splines;

    // Save the best spline
    best_spline = &(splines_[0]);
    selected_id_ = best_spline->id_;
    PRM_LOG("Selected Spline with ID: " << selected_id_);
  }

  void GlobalGuidance::Reset()
  {
    PRM_LOG("Reset()");

    // Forget paths
    path_id_was_known_ = std::vector<bool>(config_->n_paths_, false);
    known_paths_.clear();

    paths_.clear();
    splines_.clear();

    prm_.Reset();

    for (auto &obstacle : obstacles_) // Ensure that the obstacles have long enough predictions
    {
      obstacle.positions_.resize(Config::N + 1);
      for (int k = 0; k <= Config::N; k++)
        obstacle.positions_[k] = Eigen::Vector2d(100., 100.);

      obstacle.radius_ = 0.;
    }

    // selected_spline_.reset(); // Remove any previous spline references
    selected_id_ = -1;
  }

  CubicSpline3D &GlobalGuidance::GetGuidanceTrajectory(int spline_id)
  {
    if (spline_id >= (int)splines_.size())
      ROS_WARN("Trying to retrieve a spline that does not exist!");

    if (splines_.size() == 0) // selected_spline_ == nullptr) // If there is no spline - we return an "empty" trajectory
                              // to be able to keep running
    {
      if (no_message_sent_yet_)
      {
        ROS_WARN("Returning zero trajectory (no path was found)");
        no_message_sent_yet_ = false;
      }

      return CubicSpline3D::Empty(start_, config_.get());
    }

    return splines_[spline_id]; // Return the guidance trajectory
  }

  bool GlobalGuidance::checkGuidanceVisible(int id){
    PRM_LOG("Path " << sorted_indices[id] << " size: " << paths_[sorted_indices[id]].nodes_.size());
    for (size_t i = 0; i < paths_[sorted_indices[id]].nodes_.size()-1; i++){
      prm_.isVisiblePoints(paths_[sorted_indices[id]].nodes_[i]->point_, paths_[sorted_indices[id]].nodes_[i+1]->point_);
    }
    return true;
  }


  std::vector<bool> GlobalGuidance::LeftPassingH(int spline_id, Eigen::Vector2d goal)
  {
    if (spline_id >= (int)paths_.size())
    {
      ROS_WARN("Trying to get the cost of a path that does not exist!");
      std::vector<bool> empty;
      return empty;
    }
    GuidancePlanner::GeometricPath h_def_path = this->paths_[sorted_indices[spline_id]];
    // Add a common goal to close the loop with every path
    GuidancePlanner::SpaceTimePoint goal_point(goal.x(), goal.y(), h_def_path.nodes_.back()->point_.Time());
    GuidancePlanner::Node goal_node(h_def_path.nodes_.back()->id_, goal_point, h_def_path.nodes_.back()->type_);
    h_def_path.nodes_.push_back(&goal_node);
    return this->prm_.LeftPassingH(h_def_path); // Return the guidance trajectory
  }

  double GlobalGuidance::GetHomotopicCost(int spline_id, const GeometricPath &path)
  {
    if (spline_id >= (int)paths_.size())
    {
      ROS_WARN("Trying to get the cost of a path that does not exist!");
      return -1;
    }

    return this->prm_.GetHomotopicCost(this->paths_[sorted_indices[spline_id]], path); // Return the guidance trajectory
  }

  int GlobalGuidance::GetUsedTrajectory() const { return selected_id_; }

  void GlobalGuidance::SetUsedTrajectory(int spline_id) { selected_id_ = spline_id; }

  int GlobalGuidance::NumberOfGuidanceTrajectories() const { return splines_.size(); }

  int GlobalGuidance::GetIdSamePath(const GeometricPath &path)
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

  void GlobalGuidance::Visualize()
  {
    // Visualize the method per component
    VisualizeObstacles();
    prm_.Visualize();
    VisualizeGeometricPaths();
    VisualizeTrajectories();
    VisualizeSplinePoints();
    VisualizeDebug();
  }

  void GlobalGuidance::VisualizePath(GeometricPath & path){
    // Geometric Paths
    RosTools::ROSLine &path_line = ros_path_visuals_->getNewLine();
    path_line.setScale(0.15, 0.15, 0.15);
    path_line.setColorInt(path.association_.id_, config_->n_paths_, 0.75);

    Node *prev_node;
    bool first_node = true;
    for (auto &node : path.nodes_)
    {
      if (!first_node)
      {
        path_line.addLine(node->point_.MapToTime(), prev_node->point_.MapToTime());
      }
      else
      {
        first_node = false;
      }

      prev_node = node;
    }
    ros_path_visuals_->publish();
  }

  void GlobalGuidance::VisualizeGeometricPaths()
  {
    // Geometric Paths
    RosTools::ROSLine &path_line = ros_path_visuals_->getNewLine();
    path_line.setScale(0.15, 0.15, 0.15);

    for (auto &path : paths_)
    {
      path_line.setColorInt(path.association_.id_, config_->n_paths_, 0.75);

      Node *prev_node;
      bool first_node = true;
      for (auto &node : path.nodes_)
      {
        if (!first_node)
        {
          path_line.addLine(node->point_.MapToTime(), prev_node->point_.MapToTime());
        }
        else
        {
          first_node = false;
        }

        prev_node = node;
      }
    }
    ros_path_visuals_->publish();
  }

  void GlobalGuidance::VisualizeSplinePoints()
  {
    // CONTROL POINTS OF CUBIC SPLINES
    for (auto &spline : splines_)
      spline.Visualize(ros_bspline_visuals_.get());

    ros_bspline_visuals_->publish();
  }

  void GlobalGuidance::VisualizeObstacles()
  {
    RosTools::ROSPointMarker &disc = ros_obstacle_visuals_->getNewPointMarker("CYLINDER");

    // Visualize the obstacles
    int j = 0;
    for (auto &obstacle : obstacles_)
    {
      disc.setScale(obstacle.radius_ * 2., obstacle.radius_ * 2., Config::DT);
      for (int k = 0; k < Config::N; k++)
      {
        // Transparent
        disc.setColorInt(obstacle.id_, (1. - config_->visuals_transparency_) * std::pow(((double)(Config::N - k)) / (double)Config::N, 2.), RosTools::Colormap::BRUNO);
        // -> disc.setColorInt(obstacle.id_, 0.15 * std::pow(((double)(Config::N - k)) /
        // (double)Config::N, 2.), Colormap::BRUNO);

        // Largely non-transparent
        //   disc.setColorInt(j, obstacles_.size(),
        //                    0.75 * std::pow(((double)(Config::N - k)) / (double)Config::N, 2.),
        //                    Colormap::BRUNO);

        disc.addPointMarker(Eigen::Vector3d(obstacle.positions_[k](0), obstacle.positions_[k](1), (float)k * Config::DT));
      }

      j++;
    }
    ros_obstacle_visuals_->publish();
  }

  void GlobalGuidance::VisualizeTrajectories()
  {
    // Trajectories
    RosTools::ROSLine &selected_line = ros_selected_visuals_->getNewLine();
    selected_line.setScale(0.25, 0.25);

    RosTools::ROSLine &line = ros_visuals_->getNewLine();
    line.setScale(0.3, 0.3);

    RosTools::ROSPointMarker trajectory_spheres = ros_visuals_->getNewPointMarker("SPHERE");
    trajectory_spheres.setScale(0.20, 0.20, 0.20);

    RosTools::ROSTextMarker text_marker = ros_visuals_->getNewTextMarker();
    text_marker.setScale(1.0);

    bool visualize_trajectory_spheres = false;

    for (auto &spline : splines_)
    {
      std::vector<Eigen::Vector3d> &points = spline.GetSamples(); // Get samples on the current spline
      bool text_added = false;
      text_marker.setText(std::to_string(spline.id_)); // Add the spline number
      line.setColorInt(spline.id_, config_->n_paths_, 0.75);
      trajectory_spheres.setColorInt(spline.id_, config_->n_paths_, 1.0);
      line.setScale(0.15, 0.15);

      // Draw a line for the time scaled points
      for (size_t j = 0; j < points.size(); j++)
      {
        Eigen::Vector3d cur_vec = points[j];

        if (j > 0)
        {
          Eigen::Vector3d prev_vec = points[j - 1];

          if (spline.id_ == selected_id_) // highlight the selected spline
          {
            // selected_line.setColorInt(spline.id_, config_->n_paths_, 1.0);
            selected_line.setColorInt(2.0, 1.0, RosTools::Colormap::BRUNO);
            selected_line.addLine(prev_vec, cur_vec);
            // line.addLine(prev_vec, cur_vec); // Also add a regular line

            if (visualize_trajectory_spheres && j % 10 == 0)
              trajectory_spheres.addPointMarker(cur_vec);

            text_marker.setColorInt(2.0, 1.0, RosTools::Colormap::BRUNO);
          }
          else
          {
            line.addLine(prev_vec, cur_vec);

            if (visualize_trajectory_spheres && j % 10 == 0)
              trajectory_spheres.addPointMarker(cur_vec);

            text_marker.setColorInt(spline.id_, config_->n_paths_, 1.0);
          }

          if (!text_added && (double)j / (double)points.size() > 0.5)
          {
            text_marker.addPointMarker(cur_vec + Eigen::Vector3d(3.0, 0., 3.0));
            text_added = true;
          }
        }
      }
    }
    ros_selected_visuals_->publish();
    ros_visuals_->publish();
  }

  void GlobalGuidance::VisualizeDebug()
  {
    if (splines_.size() != 0)
    {
      // VISUALIZE THE USED PART OF THE SPLINE //
      RosTools::ROSLine &guidance_path = ros_guidance_path_visuals_->getNewLine();

      if (splines_.size() > 0)
      {
        guidance_path.setScale(0.15, 0.15);

        int n_segments = 20;
        double step = 0.1;
        int offset = 0; // The first segment is indexed as such

        RosTools::CubicSpline2D<tk::spline> &selected_spline = splines_[0].GetPath();

        // Go through all used segments and plot them
        for (int i = offset; i < std::min(offset + n_segments, selected_spline.NumberOfSegments()); i++)
        {
          double start_segment = selected_spline.GetSplineStart(i);
          double end_segment = selected_spline.GetSplineEnd(i);

          guidance_path.setColorInt(i, 10, 0.7);

          for (double t = start_segment + step; t <= end_segment; t += step)
          {
            guidance_path.addLine(RosTools::AsVector3d(selected_spline.GetPoint(t), 0.1), RosTools::AsVector3d(selected_spline.GetPoint(t - step), 0.1));
          }
        }
      }
    }

    ros_guidance_path_visuals_->publish();
  }

  void GlobalGuidance::ExportData(RosTools::DataSaver &data_saver) // Export data for analysis
  {
    data_saver.AddData("n_paths", paths_.size());
  }

  // Mainly for debugging purposes (not in the namespace, to use lmpcc stuff)
  void GlobalGuidance::ReconfigureCallback(GuidancePlannerConfig &config, uint32_t level)
  {
    if (first_reconfigure_callback_) // Set the reconfiguration parameters to match the yaml configuration at startup
    {
      first_reconfigure_callback_ = false;

      config.debug = Config::debug_output_;

      config.n_paths = config_->n_paths_;
      config.n_samples = config_->n_samples_;
      config.visualize_samples = config_->visualize_all_samples_;
      config.use_learning = config_->use_learning;
    }

    Config::debug_output_ = config.debug;

    config_->n_paths_ = config.n_paths;
    config_->n_samples_ = config.n_samples;
    config_->visualize_all_samples_ = config.visualize_samples;
    config_->use_learning = config.use_learning;
  }

} // namespace GuidancePlanner
