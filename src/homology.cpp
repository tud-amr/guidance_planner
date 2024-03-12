#include "guidance_planner/homology.h"

#include <ros_tools/visuals.h>

#include <guidance_planner/environment.h>

namespace GuidancePlanner
{
  Homology::Homology(bool assume_constant_velocity)
      : assume_constant_velocity_(assume_constant_velocity)
  {
    // Initialize workspace, parameters, functions
    // For multithreading!
    gsl_ws_.resize(8);
    gsl_params_.resize(8);
    gsl_f_.resize(8);
    for (int i = 0; i < 8; i++)
    {
      gsl_ws_[i] = gsl_integration_workspace_alloc(GSL_POINTS);
      gsl_params_[i].homology_class = this;

      gsl_f_[i].function = &Homology::GSLHValue;
      gsl_f_[i].params = &gsl_params_[i];
    }

    fraction_ = 1. / ((double)Config::N);
  }

  Homology::~Homology()
  {

    for (int i = 0; i < 8; i++)
    {
      gsl_integration_workspace_free(gsl_ws_[i]);
    }
  }

  /** @brief: https://link.springer.com/article/10.1007/s10514-012-9304-1 */
  bool Homology::AreEquivalent(const GeometricPath &a, const GeometricPath &b, Environment &environment, bool compute_all)
  {
    bool equivalent = true;
    // Retrieve cached h values over the obstacles (i.e., a vector)
    std::vector<double> &cached_a = cached_values_[a]; // Note: will create the cache if it does not exist
    std::vector<double> &cached_b = cached_values_[b]; // Note: will create the cache if it does not exist

    auto &obstacles = environment.GetDynamicObstacles();
    double h;

    ComputeObstacleLoops(obstacles);

    // For each obstacle
    for (size_t obstacle_id = 0; obstacle_id < obstacles.size(); obstacle_id++)
    {
      auto &obstacle = obstacles[obstacle_id];
      LoadObstacle(obstacle_id, obstacle);

      // Initialize the integration
      h = 0;
      h += PathHValue(a, cached_a, obstacle_id, obstacle); // Integrate over path A

      // Connect end points of a and b
      {
        gsl_params_[0].start = a.nodes_.back()->point_.vec;
        gsl_params_[0].end = b.nodes_.back()->point_.vec;

        // double result = 0.;
        // NumericalIntegration(result, &gsl_params_[0]);

        double result, error;
        gsl_integration_qag(&gsl_f_[0], 0, 1, GSL_ACCURACY, 0, GSL_POINTS, GSL_INTEG_GAUSS15, gsl_ws_[0], &result, &error);

        h += result;
      }

      h -= PathHValue(b, cached_b, obstacle_id, obstacle); // Integrate over path B

      // If it is not zero, then these paths are homology distinct!
      if (std::abs(h) >= 1e-1)
      {
        if (!compute_all)
          return false;
        else
          equivalent = false;
      }
      // If is zero, keep checking the other obstacles
    }

    return equivalent; // If none of them are distinct, then the two paths are homology equivalent!
  }

  /** @brief: https://link.springer.com/article/10.1007/s10514-012-9304-1 */
  double Homology::GetCost(const GeometricPath &a, const GeometricPath &b, Environment &environment)
  {
    // Retrieve cached h values over the obstacles (i.e., a vector)
    std::vector<double> &cached_a = cached_values_[a]; // Note: will create the cache if it does not exist
    std::vector<double> &cached_b = cached_values_[b]; // Note: will create the cache if it does not exist

    auto &obstacles = environment.GetDynamicObstacles();
    double h, h_total = 0;

    ComputeObstacleLoops(obstacles);

    // For each obstacle
    for (size_t obstacle_id = 0; obstacle_id < obstacles.size(); obstacle_id++)
    {
      auto &obstacle = obstacles[obstacle_id];
      LoadObstacle(obstacle_id, obstacle);

      // Initialize the integration
      h = 0;
      h += PathHValue(a, cached_a, obstacle_id, obstacle); // Integrate over path A

      // Connect end points of a and b
      {
        gsl_params_[0].start = a.nodes_.back()->point_.vec;
        gsl_params_[0].end = b.nodes_.back()->point_.vec;

        double result, error;

        gsl_integration_qag(&gsl_f_[0], 0, 1, GSL_ACCURACY, 0, GSL_POINTS, GSL_INTEG_GAUSS15, gsl_ws_[0], &result, &error);

        h += result;
      }

      h -= PathHValue(b, cached_b, obstacle_id, obstacle); // Integrate over path B
      h_total += abs(h);
      // If is zero, keep checking the other obstacles
    }

    return sqrt(h_total); // If none of them are distinct, then the two paths are homology equivalent!
  }

  std::vector<bool> Homology::LeftPassingVector(const GeometricPath &path, Environment &environment)
  {
    auto &obstacles = environment.GetDynamicObstacles();
    std::vector<double> results_h(obstacles.size());
    std::vector<bool> results(obstacles.size());

    std::list<Node> always_left_nodes;
    std::vector<Node *> always_left_node_ptrs;

    double max_time = path.nodes_.back()->point_.Time();

    // In principle we could pass the pointer, but this is clearer
    always_left_nodes.emplace_back(*(path.nodes_[0])); // Start in the same point
    // Go down
    always_left_nodes.emplace_back(1, SpaceTimePoint(path.nodes_[0]->point_.Pos()(0), path.nodes_[0]->point_.Pos()(1), -0.5), NodeType::NONE);

    // Go back
    // always_left_nodes.emplace_back(2, SpaceTimePoint(path.nodes_[0]->point_.Pos()(0) - 100, path.nodes_[0]->point_.Pos()(1), -0.5), NodeType::NONE);
    // Go left = move towards the negative y
    // always_left_nodes.emplace_back(3, SpaceTimePoint(path.nodes_[0]->point_.Pos()(0) - 100, path.nodes_[0]->point_.Pos()(1) - 100, -0.5), NodeType::NONE);
    // // Go up
    // always_left_nodes.emplace_back(4, SpaceTimePoint(path.nodes_[0]->point_.Pos()(0) - 100, path.nodes_[0]->point_.Pos()(1) - 100, max_time + 0.5), NodeType::NONE);
    // // Go front
    // always_left_nodes.emplace_back(5, SpaceTimePoint(path.nodes_.back()->point_.Pos()(0) + 100, path.nodes_[0]->point_.Pos()(1) - 100, max_time + 0.5), NodeType::NONE);
    // // Go to the goal position in y, remaining above and in front of it
    // always_left_nodes.emplace_back(6, SpaceTimePoint(path.nodes_.back()->point_.Pos()(0) + 100, path.nodes_.back()->point_.Pos()(1), max_time + 0.5), NodeType::NONE);
    // // Go to the goal position in (x, y), remaining just above it
    // always_left_nodes.emplace_back(7, SpaceTimePoint(path.nodes_.back()->point_.Pos()(0), path.nodes_.back()->point_.Pos()(1), max_time + 0.5), NodeType::NONE);
    // Go left = move towards the negative y
    always_left_nodes.emplace_back(2, SpaceTimePoint(path.nodes_[0]->point_.Pos()(0), path.nodes_[0]->point_.Pos()(1) - 100, -0.5), NodeType::NONE);
    // Go up
    always_left_nodes.emplace_back(3, SpaceTimePoint(path.nodes_[0]->point_.Pos()(0), path.nodes_[0]->point_.Pos()(1) - 100, max_time + 0.5), NodeType::NONE);
    // Go to the goal position in (x)
    always_left_nodes.emplace_back(4, SpaceTimePoint(path.nodes_.back()->point_.Pos()(0), path.nodes_[0]->point_.Pos()(1) - 100, max_time + 0.5), NodeType::NONE);
    // Go to the goal position in (x, y), remaining just above it
    always_left_nodes.emplace_back(5, SpaceTimePoint(path.nodes_.back()->point_.Pos()(0), path.nodes_.back()->point_.Pos()(1), max_time + 0.5), NodeType::NONE);
    // always_left_nodes.emplace_back(2, SpaceTimePoint(path.nodes_[0]->point_.Pos()(0), path.nodes_[0]->point_.Pos()(1) - 100, -0.5), NodeType::NONE);
    // always_left_nodes.emplace_back(3, SpaceTimePoint(path.nodes_[0]->point_.Pos()(0), path.nodes_[0]->point_.Pos()(1) - 100, max_time + 0.5), NodeType::NONE);
    // always_left_nodes.emplace_back(4, SpaceTimePoint(path.nodes_.back()->point_.Pos()(0), path.nodes_.back()->point_.Pos()(1), max_time + 0.5), NodeType::NONE);
    // Connect to the given path
    always_left_nodes.emplace_back(*(path.nodes_.back()));

    for (auto &node : always_left_nodes)
      always_left_node_ptrs.push_back(&node);

    GeometricPath always_left_path(always_left_node_ptrs);

    AreEquivalent(always_left_path, path, environment, true);

    // Retrieve the values from the cache
    std::vector<double> &cached_a = cached_values_[always_left_path]; // Note: will create the cache if it does not exist
    std::vector<double> &cached_b = cached_values_[path];             // Note: will create the cache if it does not exist

    for (size_t i = 0; i < cached_a.size(); i++)
    {
      results_h[i] = cached_a[i] - cached_b[i];
      results[i] = std::abs(results_h[i]) < 0.1; // If we are in the same homology as "always left", then we are going left here
    }

    return results;
  }

  double Homology::GSLHValue(double lambda, void *params)
  {
    GSLParams *p = (GSLParams *)params; // Cast to correct parameters

    // Given this lambda
    Eigen::Vector3d r = Homology::Line(p->start, p->end, lambda);
    Eigen::Vector3d dr = p->end - p->start;

    return p->homology_class->ObstacleHValue(r, dr);
  }

  double Homology::ObstacleHValue(const Eigen::Vector3d &r, const Eigen::Vector3d &dr)
  {
    double h = 0;

    // H-signature over the segments in the obstacle skeleton from start to end
    for (size_t obstacle_segment = 1; obstacle_segment < obstacle_segments_.size(); obstacle_segment++)
      h += SegmentHValue(obstacle_segments_[obstacle_segment - 1], obstacle_segments_[obstacle_segment], r, dr);

    h += SegmentHValue(obstacle_segments_.back(), obstacle_segments_[0], r, dr); // Connect the end and the start

    // h += SegmentHValue(obstacle_p1_, obstacle_p2_, r, dr); // The obstacle itself
    // h += SegmentHValue(obstacle_p2_, obstacle_p3_, r, dr); // A line above the state-space
    // h += SegmentHValue(obstacle_p3_, obstacle_p4_, r, dr); // A downwards line
    // h += SegmentHValue(obstacle_p4_, obstacle_p1_, r, dr); // Close the loop
    return h * (1. / (4. * M_PI));
  }

  // Start and end relate to obstacle start and end positions of an obstacle segment
  double Homology::SegmentHValue(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const Eigen::Vector3d &r, const Eigen::Vector3d &dr)
  {
    Eigen::Vector3d p_i = start - r;
    Eigen::Vector3d p_j = end - r;

    Eigen::Vector3d d = (end - start).cross(p_i.cross(p_j)) / std::pow((end - start).norm(), 2.);
    Eigen::Vector3d h = 1. / std::pow(d.norm(), 2.) * (d.cross(p_j) / p_j.norm() - d.cross(p_i) / p_i.norm());
    return h.transpose() * dr;
  }

  void Homology::LoadObstacle(int i, const Obstacle &obstacle)
  {
    obstacle_segments_ = obstacle_points_[i];
    // obstacle_p1_ = obstacle_points_[0][i];
    // obstacle_p2_ = obstacle_points_[1][i];
    // obstacle_p3_ = obstacle_points_[2][i];
    // obstacle_p4_ = obstacle_points_[3][i];
  }

  void Homology::ComputeObstacleLoops(const std::vector<Obstacle> &obstacles)
  {
    if (!obstacle_points_ready_ && obstacles.size() > 0)
    {
      obstacle_points_.resize(obstacles.size());
      for (size_t o = 0; o < obstacle_points_.size(); o++)
      {
        obstacle_points_[o].clear();
        obstacle_points_[o].reserve(obstacles[o].positions_.size() + 4); // Positions is from 0 to N, we need 0 / 1 - N / +
      }

      // Retrieve and set the obstacle skeleton
      for (size_t o = 0; o < obstacles.size(); o++)
      {
        const auto &obstacle = obstacles[o];

        Eigen::Vector3d start(obstacle.positions_[0](0), obstacle.positions_[0](1), 0);
        // Eigen::Vector3d end(obstacle.positions_.back()(0), obstacle.positions_.back()(1), Config::N);
        obstacle_points_[o].push_back(Eigen::Vector3d(start(0), start(1), -1)); // A start below the actual start

        if (!assume_constant_velocity_)
        {
          for (size_t p = 0; p < obstacle.positions_.size(); p++) // Add the obstacle predictions, skipping the current position
            obstacle_points_[o].push_back(Eigen::Vector3d(obstacle.positions_[p](0), obstacle.positions_[p](1), p));
        }
        else
        {
          // For constant velocity, only one line is sufficient
          obstacle_points_[o].push_back(Eigen::Vector3d(obstacle.positions_.back()(0), obstacle.positions_.back()(1), obstacle.positions_.size() - 1));
        }
        // obstacle_points_[o].push_back(Line(start, end, -fraction_)); // A start below the actual start
        // obstacle_points_[0][i] = Line(start, end, -fraction_);                            // A start below the actual start
        // obstacle_points_[1][i] = Line(start, end, 1 + fraction_);                         // An end above the actual end
        // obstacle_points_[2][i] = obstacle_points_[1][i] + Eigen::Vector3d(-250., 0., 0.); // Move outside of the region

        obstacle_points_[o].push_back(obstacle_points_[o].back() + Eigen::Vector3d(0., 0., 1.));    // An end above the actual end
        obstacle_points_[o].push_back(obstacle_points_[o].back() + Eigen::Vector3d(-250., 0., 0.)); // Move outside of the region
        obstacle_points_[o].push_back(obstacle_points_[o].back());                                  // In that same (x,y) move down
        obstacle_points_[o].back()(2) = obstacle_points_[o][0](2);
        // obstacle_points_[3][i] = obstacle_points_[2][i];
        // obstacle_points_[3][i](2) = obstacle_points_[0][i](2); // Move down
      }
      obstacle_points_ready_ = true;
    }
  }

  double Homology::PathHValue(const GeometricPath &path, std::vector<double> &cached_h, const int obstacle_id, const Obstacle &obstacle)
  {
    if (obstacle_id < (int)cached_h.size())
      return cached_h[obstacle_id]; // Retrieve from cache

    std::vector<double> results(path.nodes_.size() - 1, 0);
    // #pragma omp parallel for num_threads(8)
    for (size_t n = 1; n < path.nodes_.size(); n++) // From the start to the end of a
    {
      double error;

      // #ifdef _OPENMP
      int thread_id = omp_get_thread_num();
      // #else
      // int thread_id = 0;
      // #endif
      gsl_params_[thread_id].start = path.nodes_[n - 1]->point_.vec;
      gsl_params_[thread_id].end = path.nodes_[n]->point_.vec;

      // NumericalIntegration(results[n - 1], &gsl_params_[thread_id]);

      gsl_integration_qag(&gsl_f_[thread_id], 0, 1, GSL_ACCURACY, 0, GSL_POINTS, GSL_INTEG_GAUSS15, gsl_ws_[thread_id], &results[n - 1], &error);
    }

    double h_value = 0;
    for (auto &partial_result : results)
      h_value += partial_result;

    cached_h.push_back(h_value); // Save in the cache
    return h_value;
  }

  inline void Homology::NumericalIntegration(double &result, void *params)
  {
    double num = 25.;
    result += Homology::GSLHValue(0., params) * 0.5;
    result += Homology::GSLHValue(1., params) * 0.5;

    for (double l = (1. / num); l < 1.; l += (1. / num))
      result += Homology::GSLHValue(l, params);

    result *= (1. / num);
  }

  void Homology::Visualize(Environment &environment)
  {
    auto &debug_visuals = VISUALS.getPublisher("guidance_planner/homology");
    auto &line = debug_visuals.getNewLine();
    line.setScale(0.05);

    const auto &obstacles = environment.GetDynamicObstacles();
    for (size_t i = 0; i < obstacles.size(); i++)
    {
      const auto &obstacle = obstacles[i];
      line.setColorInt(obstacle.id_, 1., RosTools::Colormap::BRUNO);

      LoadObstacle(i, obstacle);

      for (auto &p : obstacle_segments_)
        p(2) *= Config::DT; // Scale the time axis

      for (size_t p = 1; p < obstacle_segments_.size(); p++)
        line.addLine(obstacle_segments_[p - 1], obstacle_segments_[p]);
      // line.addBrokenLine(obstacle_segments_[p - 1], obstacle_segments_[p], 1.);

      line.addLine(obstacle_segments_.back(), obstacle_segments_.front());
      // line.addBrokenLine(obstacle_segments_.back(), obstacle_segments_.front(), 1.);

      // obstacle_p1_(2) *= Config::DT;
      // obstacle_p2_(2) *= Config::DT;
      // obstacle_p3_(2) *= Config::DT;
      // obstacle_p4_(2) *= Config::DT;

      // line.addBrokenLine(obstacle_p1_, obstacle_p2_, 1.);
      // line.addBrokenLine(obstacle_p2_, obstacle_p3_, 1.);
      // line.addBrokenLine(obstacle_p3_, obstacle_p4_, 1.);
      // line.addBrokenLine(obstacle_p4_, obstacle_p1_, 1.);
    }

    debug_visuals.publish();
  }

  bool operator==(const GeometricPath &a, const GeometricPath &b)
  {
    {
      if (a.nodes_.size() != b.nodes_.size())
        return false;

      for (size_t i = 0; i < a.nodes_.size(); i++)
      {
        // If we do not have two goals (they are always equal)
        if (!(a.nodes_[i]->type_ == NodeType::GOAL && b.nodes_[i]->type_ == NodeType::GOAL))
        {
          if (a.nodes_[i]->id_ != b.nodes_[i]->id_) // Then if they do not have the same ID, these are not the same paths!
            return false;
        }
      }

      return true;
    }
  }
} // namespace Homotopy
