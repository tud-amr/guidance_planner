#include "guidance_planner/global_guidance.h"

namespace GuidancePlanner
{

    GlobalGuidance::~GlobalGuidance()
    {
    }

    GlobalGuidance::GlobalGuidance()
    {
        PRM_LOG("Initializing Global Guidance");
        config_.reset(new HomotopyConfig());
        prm_.Init(nh_, config_.get());

        /* Initialize visuals */
        ros_visuals_.reset(new ROSMarkerPublisher(nh_, "lmpcc/homotopy/guidance_trajectories", "map", 500));
        ros_bspline_visuals_.reset(new ROSMarkerPublisher(nh_, "lmpcc/homotopy/spline_points", "map", 200));
        ros_selected_visuals_.reset(new ROSMarkerPublisher(nh_, "lmpcc/homotopy/selected_guidance", "map", 200));
        ros_guidance_path_visuals_.reset(new ROSMarkerPublisher(nh_, "lmpcc/homotopy/guidance_path", "map", 200));
        ros_obstacle_visuals_.reset(new ROSMarkerPublisher(nh_, "lmpcc/homotopy/obstacles_3d", "map", 200));
        ros_path_visuals_.reset(new ROSMarkerPublisher(nh_, "lmpcc/homotopy/geometric_paths", "map", 200));

        /* Initialize benchmarkers for debugging purposes */
        benchmarkers_.push_back(std::unique_ptr<Helpers::Benchmarker>(new Helpers::Benchmarker("Homotopy", false, 0)));
        benchmarkers_.push_back(std::unique_ptr<Helpers::Benchmarker>(new Helpers::Benchmarker("Visibility-PRM", false, 0)));
        benchmarkers_.push_back(std::unique_ptr<Helpers::Benchmarker>(new Helpers::Benchmarker("Search", false, 0)));
        benchmarkers_.push_back(std::unique_ptr<Helpers::Benchmarker>(new Helpers::Benchmarker("Bsplines", false, 0)));

        first_reconfigure_callback_ = true;

        start_velocity_ = Eigen::Vector2d(0., 0.);

        Reset();
    }

    void GlobalGuidance::LoadObstacles(const std::vector<Obstacle> &obstacles, const std::vector<std::vector<Halfspace>>& static_obstacles)
    {
        obstacles_ = obstacles;
        static_obstacles_ = static_obstacles;
    }

    void GlobalGuidance::SetStart(const Eigen::Vector2d &start, const double orientation, const double velocity)
    {
        start_ = start;
        orientation_ = orientation;
        start_velocity_ = Eigen::Vector2d(velocity * std::cos(orientation), velocity * std::sin(orientation));
    }

    void GlobalGuidance::SetGoals(const std::vector<Eigen::Vector2d> &goals, const std::vector<double> &goal_costs)
    {
        goals_ = goals;
        goal_costs_ = goal_costs;
    }

    bool GlobalGuidance::Update()
    {
        no_message_sent_yet_ = true;
        benchmarkers_[0]->start();

        /* Verify validity of input data */
        for (auto &obstacle : obstacles_) // Dynamic obstacles
            LMPCC_ASSERT((int)obstacle.positions_.size() >= HomotopyConfig::N + 1, "Obstacles should have their predictions populated from 0-N");

        // Static obstacles
        // LMPCC_ASSERT(std::abs(data_ptr_->halfspaces_[0][0].A_(0) - data_ptr_->halfspaces_[1][0].A_(0)) < 1e-8, "We assume static halfspaces to be the same for all timesteps (violated).");

        PRM_LOG("======== PRM ==========");

        prm_.LoadData(static_obstacles_,
                      obstacles_,
                      start_, orientation_, start_velocity_,
                      goals_,
                      goal_costs_,
                      selected_id_);

        benchmarkers_[1]->start();
        Graph &graph = prm_.Update(); // Construct a graph using visibility PRM
        benchmarkers_[1]->stop();

        PRM_LOG("======== Path Search ==========");
        benchmarkers_[2]->start();

        paths_.clear();
        for (auto &goal : graph.goal_nodes_)
        {
            std::vector<Node *> L = {graph.start_node_};

            std::vector<GeometricPath> cur_paths;
            graph_search_.Search(graph, config_->n_paths_, L, cur_paths, goal); // graph.goal_nodes_[0]); // Find paths via a graph-search

            for (auto &path : cur_paths)
            {
                paths_.emplace_back(path); // Add all current paths
            }
        }

        FilterPaths(); // Remove any poorly formed paths

        RemoveHomotopicEquivalentPaths(); // Remove paths in duplicate topologies

        /* Select the best paths*/
        std::sort(paths_.begin(), paths_.end(), [](const GeometricPath &a, const GeometricPath &b)
                  { return 1000 * a.nodes_.back()->point_.Pos()(0) - a.Length3D() > 1000 * b.nodes_.back()->point_.Pos()(0) - b.Length3D(); });

        paths_.resize(std::min((int)paths_.size(), config_->n_paths_));

        // If there are no paths - stop
        if (paths_.size() == 0)
        {
            /*for (auto &benchmarker : benchmarkers_)
                if (benchmarker->isRunning())
                    benchmarker->stop();

            splines_.clear();
            prm_.Reset();*/

            PRM_WARN("Guidance failed to find a path from the robot position to the goal (using last path)");

            // return false;
        }

        // Print all paths
        PRM_LOG("Paths:");
        for (auto &path : paths_)
            PRM_LOG("\t" << path << "\b");

        PRM_LOG("======== Association ==========");
        // Association Step 1: Check the segment association IDs of the nodes in each path and assign an ID to each Path based on it
        // While each node connects only to two guards, a node can appear in more than one path, due to the path search.
        // A path is therefore associated with a sequence of nodes.
        // To maintain a consistent ID for nodes throughout replacement operations, etc., we use a separate ID: segment_association_id_

        // Create a list of indices to be used for the paths
        std::vector<int> available_ids(config_->n_paths_); // NOTE: In some cases the previous path ID may be higher than the number of paths we have currently, that's fine
        std::iota(available_ids.begin(), available_ids.end(), 0);

        for (auto &path : paths_)
        {
            LMPCC_ASSERT(path.association_.AllSegmentsAssigned(), "A segment in this path was missing a segment ID"); // By construction all segments should have an ID
            for (auto &previous_path_assocation : known_paths_)
            {
                if (path.association_.Matches(previous_path_assocation))
                {
                    PRM_LOG("Path " << path << " matches the previously known path with ID: " << previous_path_assocation.id_);
                    path.association_.id_ = previous_path_assocation.id_;
                    path_id_was_known_[path.association_.id_] = true; // Ensures that this path was not randomly given the ID - but obtained it

                    auto iterator_at_available_id = std::find(available_ids.begin(), available_ids.end(), path.association_.id_);           // Should be updated after erasing
                    LMPCC_ASSERT(iterator_at_available_id != std::end(available_ids), "When assigning path IDs, an ID was assigned twice"); // This ID must still be available

                    available_ids.erase(iterator_at_available_id); // The ID is not available anymore

                    break;
                }
            }
        }

        // Now for all paths that do not have an ID assigned, assign them in order
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

        // Association Step 2: Save the known paths via an ID and a list of segment IDs
        known_paths_.clear();
        for (auto &path : paths_)
        {
            known_paths_.push_back(path.association_); // Only save the assocation of each path (we will need it in the next iteration)
            PRM_LOG("Saving path [" << known_paths_.back().id_ << "]: " << path);
        }

        /** Transfer the path association information to the nodes in it AND propagate the nodes to the next iteration */
        prm_.TransferPathInformationAndPropagate(paths_, known_paths_);
        benchmarkers_[2]->stop();

        // For each path, create and optimize a spline
        PRM_LOG("======== Cubic Splines ==========");

        if (paths_.size() == 0) // Skip splines if no paths were found, so that we can still use the previous splines
        {
            benchmarkers_[0]->stop();

            return false;
        }

        benchmarkers_[3]->start();

        splines_.clear();

        for (auto &path : paths_)
        {
            // Fit Cubic-Splines for each path
            splines_.emplace_back(path, config_.get(), start_velocity_);
            splines_.back().Optimize(obstacles_);
        }

        // Select the most suitable guidance trajectory
        spline_costs_.clear();
        // double min_cost = 1.0e10;
        // CubicSpline3D *best_spline = nullptr;

        for (auto &spline : splines_)
        {
            // The consistency weight decays with a logistic function
            // double consistency_weight = 1. - Helpers::LogisticFunction(1., 2., (double)selected_id_iterations_); // 1.; // Standard penalty
            double consistency_weight = 1.;

            /* The last check verifies that the new path did not by accident get assigned on the ID of the previous selected spline */
            if (spline.id_ == selected_id_ && path_id_was_known_[spline.id_])
                consistency_weight = 0.;

            // Add costs for all splines
            double spline_cost = 0.;
            spline_cost += spline.WeightPathLength() * config_->selection_weight_length_;
            spline_cost += spline.WeightLongitudinalAcceleration() * config_->selection_weight_longitudinal_acceleration_;
            spline_cost += spline.WeightLateralAcceleration() * config_->selection_weight_lateral_acceleration_;
            spline_cost += spline.WeightVelocity() * config_->selection_weight_velocity_;
            spline_cost += consistency_weight * config_->selection_weight_consistency_;

            spline_costs_.push_back(spline_cost);
            spline.stored_cost_ = spline_cost;
        }

        // Sort the spline costs from low = 0, to high = n_splines (i.e., best spline is at 0)
        std::sort(splines_.begin(), splines_.end(), [](const CubicSpline3D &a, const CubicSpline3D &b)
                  { return a.stored_cost_ < b.stored_cost_; });

        PRM_LOG("Best Spline ID: " << splines_[0].id_);

        if (selected_id_ == splines_[0].id_) // If we selected the same spline add one to its duration
            selected_id_iterations_++;
        else
            selected_id_iterations_ = 0;

        // selected_id_ = splines_[0].id_; -> Set from the homotopy guidance

        benchmarkers_[3]->stop();
        PRM_LOG("=========================");

        benchmarkers_[0]->stop();

        return true; /* Succesful running */
    }

    void GlobalGuidance::FilterPaths()
    {
        PRM_LOG("Filtering " << paths_.size() << " paths.");
        std::vector<bool> removal_markers(paths_.size(), false);
        for (size_t path_id = 0; path_id < paths_.size(); path_id++)
        {

            auto &path = paths_[path_id];
            for (size_t i = 0; i < path.nodes_.size(); i++)
            {
                auto &node = path.nodes_[i];

                if (i > 0)
                {
                    auto &prev_node = path.nodes_[i - 1];

                    // Remove paths with excessive angles in its connections (disabled)
                    // double connection_angle = std::atan2(node->point_.Pos()(1) - prev_node->point_.Pos()(1), node->point_.Pos()(0) - prev_node->point_.Pos()(0));
                    // if (std::abs(connection_angle) > M_PI_2)
                    // {
                    //     removal_markers[path_id] = true;
                    //     break;
                    // }

                    if (node->point_.Time() <= prev_node->point_.Time()) // Check causality
                    {
                        removal_markers[path_id] = true;
                        break;
                    }
                }
            }
        }

        // Remove any marked paths
        for (int path_id = (int)paths_.size() - 1; path_id >= 0; path_id--)
        {
            if (removal_markers[path_id])
                paths_.erase(paths_.begin() + path_id);
        }
    }

    void GlobalGuidance::RemoveHomotopicEquivalentPaths()
    {
        // 1) Track in how many paths each connector is present so that we may remove connectors that are not in any path in the end
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

                // If these two paths are homotopic equivalent (can also be checked by verifying that the associations are the same)
                if (paths_[i].association_.Matches(paths_[j].association_) || prm_.AreHomotopicEquivalent(paths_[i], paths_[j]))
                {
                    PRM_LOG(paths_[i] << " and " << paths_[j] << " are homotopic equivalent paths");

                    // Keep the "best" path
                    if (prm_.FirstPathIsBetter(paths_[i], paths_[j]))
                    {
                        PRM_LOG("Marked the second path for removal");

                        removal_marker[j] = true;
                        paths_[i].association_.Merge(paths_[j].association_, selected_id_); // Merge associations (preferring selected IDs)
                    }
                    else
                    {
                        PRM_LOG("Marked the first path for removal");

                        removal_marker[i] = true;
                        paths_[j].association_.Merge(paths_[i].association_, selected_id_); // Merge associations (preferring selected IDs)
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
            obstacle.positions_.resize(HomotopyConfig::N + 1);
            for (int k = 0; k <= HomotopyConfig::N; k++)
                obstacle.positions_[k] = Eigen::Vector2d(100., 100.);

            obstacle.radius_ = 0.;
        }

        // selected_spline_.reset(); // Remove any previous spline references
        selected_id_ = -1;
        selected_id_iterations_ = 0;
    }

    CubicSpline3D &GlobalGuidance::GetGuidanceTrajectory(int spline_id)
    {
        if (spline_id >= splines_.size())
            LMPCC_WARN("Trying to retrieve a spline that does not exist!");

        if (splines_.size() == 0) // selected_spline_ == nullptr) // If there is no spline - we return an "empty" trajectory to be able to keep running
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

    int GlobalGuidance::GetUsedTrajectory() const
    {
        return selected_id_;
    }

    void GlobalGuidance::SetUsedTrajectory(int spline_id)
    {
        selected_id_ = spline_id;
    }

    int GlobalGuidance::NumberOfGuidanceTrajectories() const
    {
        return splines_.size();
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

    void GlobalGuidance::VisualizeGeometricPaths()
    {
        // Geometric Paths
        ROSLine &path_line = ros_path_visuals_->getNewLine();
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
        ROSPointMarker &disc = ros_obstacle_visuals_->getNewPointMarker("CYLINDER");

        // Visualize the obstacles
        int j = 0;
        for (auto &obstacle : obstacles_)
        {
            disc.setScale(obstacle.radius_ * 2., obstacle.radius_ * 2., HomotopyConfig::DT);
            for (int k = 0; k < HomotopyConfig::N; k++)
            {
                // Transparent
                // disc.setColorInt(obstacle.id_, obstacles_.size(), 0.15 * std::pow(((double)(HomotopyConfig::N - k)) / (double)HomotopyConfig::N, 2.), Colormap::BRUNO);
                // -> disc.setColorInt(obstacle.id_, 0.15 * std::pow(((double)(HomotopyConfig::N - k)) / (double)HomotopyConfig::N, 2.), Colormap::BRUNO);

                // Largely non-transparent
                disc.setColorInt(j, obstacles_.size(), 0.75 * std::pow(((double)(HomotopyConfig::N - k)) / (double)HomotopyConfig::N, 2.), Colormap::BRUNO);

                disc.addPointMarker(Eigen::Vector3d(obstacle.positions_[k](0), obstacle.positions_[k](1), (float)k * HomotopyConfig::DT));
            }

            j++;
        }
        ros_obstacle_visuals_->publish();
    }

    void GlobalGuidance::VisualizeTrajectories()
    {
        // Trajectories
        ROSLine &selected_line = ros_selected_visuals_->getNewLine();
        selected_line.setScale(0.25, 0.25);

        ROSLine &line = ros_visuals_->getNewLine();
        line.setScale(0.3, 0.3);

        ROSPointMarker trajectory_spheres = ros_visuals_->getNewPointMarker("SPHERE");
        trajectory_spheres.setScale(0.20, 0.20, 0.20);

        ROSTextMarker text_marker = ros_visuals_->getNewTextMarker();
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
                        selected_line.setColorInt(2.0, 1.0, Colormap::BRUNO);
                        selected_line.addLine(prev_vec, cur_vec);
                        // line.addLine(prev_vec, cur_vec); // Also add a regular line

                        if (visualize_trajectory_spheres && j % 10 == 0)
                            trajectory_spheres.addPointMarker(cur_vec);

                        text_marker.setColorInt(2.0, 1.0, Colormap::BRUNO);
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
            ROSLine &guidance_path = ros_guidance_path_visuals_->getNewLine();

            if (splines_.size() > 0)
            {
                guidance_path.setScale(0.15, 0.15);

                int n_segments = 20;
                double step = 0.1;
                int offset = 0; // The first segment is indexed as such

                CubicSpline2D<tk::spline> &selected_spline = splines_[0].GetPath();

                // Go through all used segments and plot them
                for (int i = offset; i < std::min(offset + n_segments, selected_spline.NumberOfSegments()); i++)
                {
                    double start_segment = selected_spline.GetSplineStart(i);
                    double end_segment = selected_spline.GetSplineEnd(i);

                    guidance_path.setColorInt(i, 10, 0.7);

                    for (double t = start_segment + step; t <= end_segment; t += step)
                    {
                        guidance_path.addLine(Helpers::AsVector3d(selected_spline.GetPoint(t), 0.1),
                                              Helpers::AsVector3d(selected_spline.GetPoint(t - step), 0.1));
                    }
                }
            }
        }

        ros_guidance_path_visuals_->publish();
    }

    void GlobalGuidance::ExportData(DataSaver &data_saver) // Export data for analysis
    {
        int i = 0;
        for (auto &spline : splines_)
        {
            double consistency_weight = 1.; // Standard penalty

            if (spline.id_ == selected_id_)
                consistency_weight = 0.;

            data_saver.AddData("spline_cost_" + std::to_string(spline.id_), spline_costs_[i]);
            data_saver.AddData("lateral_cost_" + std::to_string(spline.id_), spline.WeightLateralAcceleration());
            data_saver.AddData("path_cost_" + std::to_string(spline.id_), spline.WeightPathLength());
            data_saver.AddData("consistency_cost_" + std::to_string(spline.id_), consistency_weight);
            i++;
        }
    }
}

// Mainly for debugging purposes (not in the namespace, to use lmpcc stuff)
// void Homotopy::GlobalGuidance::ReconfigureCallback(lmpcc::PredictiveControllerConfig &config, uint32_t level)
// {
//     if (first_reconfigure_callback_) // Set the reconfiguration parameters to match the yaml configuration at startup
//     {
//         first_reconfigure_callback_ = false;

//         config.debug = HomotopyConfig::debug_output_;

//         config.n_paths = config_->n_paths_;
//         config.n_samples = config_->n_samples_;

//         config.geometric = config_->geometric_weight_;
//         config.smoothness = config_->smoothness_weight_;
//         config.collision = config_->collision_weight_;
//         config.repeat_times = config_->repeat_times_;

//         config.spline_length = config_->selection_weight_length_;
//         config.spline_velocity = config_->selection_weight_velocity_;
//         config.spline_longitudinal_acceleration = config_->selection_weight_longitudinal_acceleration_;
//         config.spline_lateral_acceleration = config_->selection_weight_lateral_acceleration_;
//         config.spline_consistency = config_->selection_weight_consistency_;
//     }

//     HomotopyConfig::debug_output_ = config.debug;

//     config_->n_paths_ = config.n_paths;
//     config_->n_samples_ = config.n_samples;

//     config_->geometric_weight_ = config.geometric;
//     config_->smoothness_weight_ = config.smoothness;
//     config_->collision_weight_ = config.collision;
//     config_->repeat_times_ = config.repeat_times;

//     config_->selection_weight_length_ = config.spline_length;
//     config_->selection_weight_velocity_ = config.spline_velocity;
//     config_->selection_weight_longitudinal_acceleration_ = config.spline_longitudinal_acceleration;
//     config_->selection_weight_lateral_acceleration_ = config.spline_lateral_acceleration;
//     config_->selection_weight_consistency_ = config.spline_consistency;
// }