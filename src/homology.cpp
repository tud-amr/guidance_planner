#include "guidance_planner/homology.h"

namespace GuidancePlanner
{
Homology::Homology()
{
  // Initialize workspace, parameters, functions
  // For multithreading!
  gsl_ws_.resize(8);
  gsl_params_.resize(8);
  gsl_f_.resize(8);
  for (int i = 0; i < 8; i++)
  {
    gsl_ws_[i] = gsl_integration_workspace_alloc(20);
    gsl_params_[i].homology_class = this;

    gsl_f_[i].function = &Homology::GSLHValue;
    gsl_f_[i].params = &gsl_params_[i];
  }
  // gsl_params_.reset(new GSLParams()); // Is a pointer because the parameters and homology class depend on each other

  fraction_ = 1. / ((double)HomotopyConfig::N);
}

Homology::~Homology()
{

  for (int i = 0; i < 8; i++)
  {
    gsl_integration_workspace_free(gsl_ws_[i]);
  }
}

/** @brief: https://link.springer.com/article/10.1007/s10514-012-9304-1 */
bool Homology::AreEquivalent(const GeometricPath &a, const GeometricPath &b, Environment &environment)
{
  // Retrieve cached h values over the obstacles (i.e., a vector)
  std::vector<double> &cached_a = cached_values_[a]; // Note: will create the cache if it does not exist
  std::vector<double> &cached_b = cached_values_[b]; // Note: will create the cache if it does not exist

  auto &obstacles = environment.GetDynamicObstacles();
  double h;

  // For each obstacle
  for (size_t obstacle_id = 0; obstacle_id < obstacles.size(); obstacle_id++)
  {
    auto &obstacle = obstacles[obstacle_id];

    // Initialize the integration
    h = 0;

    ComputeObstacleLoop(obstacle);

    h += PathHValue(a, cached_a, obstacle_id, obstacle); // Integrate over path A

    // Connect end points of a and b
    {
      gsl_params_[0].start = a.nodes_.back()->point_.vec;
      gsl_params_[0].end = b.nodes_.back()->point_.vec;

      double result, error;

      gsl_integration_qag(&gsl_f_[0], 0, 1, 1e-1, 0, 20, GSL_INTEG_GAUSS15, gsl_ws_[0], &result, &error);

      h += result;
    }

    h -= PathHValue(b, cached_b, obstacle_id, obstacle); // Integrate over path B

    // If it is not zero, then these paths are homology distinct!
    if (std::abs(h) >= 1e-1)
      return false;

    // If is zero, keep checking the other obstacles
  }

  return true; // If none of them are distinct, then the two paths are homology equivalent!
}

std::vector<bool> Homology::PassesRight(const GeometricPath &path, Environment &environment)
{
  auto &obstacles = environment.GetDynamicObstacles();

  std::vector<bool> results(obstacles.size());

  std::vector<double> &cached = cached_values_[path]; // Note: will create the cache if it does not exist

  // For each obstacle
  for (size_t obstacle_id = 0; obstacle_id < obstacles.size(); obstacle_id++)
  {
    auto &obstacle = obstacles[obstacle_id];

    ComputeObstacleLoop(obstacle);

    double path_h_value = PathHValue(path, cached, obstacle_id, obstacle);
    std::cout << obstacle_id << ": " << path_h_value << std::endl;
    results[obstacle_id] = path_h_value > 0.1;
    // if (results[obstacle_id])
    //   std::cout << "passes right\n";
    // else
    //   std::cout << "passes left\n";
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
};

double Homology::ObstacleHValue(const Eigen::Vector3d &r, const Eigen::Vector3d &dr)
{
  double h = 0;
  h += SegmentHValue(obstacle_p1_, obstacle_p2_, r, dr); // The obstacle itself
  h += SegmentHValue(obstacle_p2_, obstacle_p3_, r, dr); // A line above the state-space
  h += SegmentHValue(obstacle_p3_, obstacle_p4_, r, dr); // A downwards line
  h += SegmentHValue(obstacle_p4_, obstacle_p1_, r, dr); // Close the loop
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

void Homology::ComputeObstacleLoop(const Obstacle &obstacle)
{
  // Retrieve and set the obstacle skeleton
  start_ = Eigen::Vector3d(obstacle.positions_[0](0), obstacle.positions_[0](1), 0);
  end_ = Eigen::Vector3d(obstacle.positions_.back()(0), obstacle.positions_.back()(1), HomotopyConfig::N);

  obstacle_p1_ = Line(start_, end_, -fraction_);               // A start below the actual start
  obstacle_p2_ = Line(start_, end_, 1 + fraction_);            // An end above the actual end
  obstacle_p3_ = obstacle_p2_ + Eigen::Vector3d(-25., 0., 0.); // Move outside of the region
  obstacle_p4_ = obstacle_p3_;
  obstacle_p4_(2) = obstacle_p1_(2); // Move down
}

double Homology::PathHValue(const GeometricPath &path, std::vector<double> &cached_h, const int obstacle_id, const Obstacle &obstacle)
{

  if (obstacle_id < (int)cached_h.size())
    return cached_h[obstacle_id]; // Retrieve from cache

  std::vector<double> results(path.nodes_.size() - 1, 0);
#pragma omp parallel for num_threads(8)
  for (size_t n = 1; n < path.nodes_.size(); n++) // From the start to the end of a
  {
    double error;

    int thread_id = omp_get_thread_num();
    gsl_params_[thread_id].start = path.nodes_[n - 1]->point_.vec;
    gsl_params_[thread_id].end = path.nodes_[n]->point_.vec;

    gsl_integration_qag(&gsl_f_[thread_id], 0, 1, 1e-1, 0, 20, GSL_INTEG_GAUSS15, gsl_ws_[thread_id], &results[n - 1], &error);
  }

  double h_value = 0;
  for (auto &partial_result : results)
    h_value += partial_result;

  cached_h.push_back(h_value); // Save in the cache
  return h_value;
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

}; // namespace Homotopy
