#include "guidance_planner/environment.h"

#include <guidance_planner/types.h>

#include <ros_tools/math.h>

namespace GuidancePlanner
{

  // REGULAR ENVIRONMENT
  void Environment::Init() {}

  bool Environment::InCollision(const SpaceTimePoint &point, double with_margin)
  {

    for (auto &obstacle : dynamic_obstacles_)
    {
      // Round the time index to the nearest integer
      if (RosTools::distance(obstacle.positions_[std::round(point.Time())], point.Pos()) < obstacle.radius_ + with_margin) // Note that the obstacle positions at k = 0 is the initial state
        return true;
    }

    // // If the time is not integer, we need to also check
    // if (point.Time() != (int)point.Time())
    // {
    // }

    for (auto &halfspace : static_obstacles_)
    {
      if (halfspace.A_.transpose() * point.Pos() > halfspace.b_)
        return true;
    }

    return false;
  }

  void Environment::LoadObstacles(const std::vector<Obstacle> &dynamic_obstacles, const std::vector<Halfspace> &static_obstacles)
  {

    dynamic_obstacles_ = dynamic_obstacles;
    static_obstacles_ = static_obstacles;
  }

  bool Environment::IsVisible(const Node &a, const Node &b) { return IsVisible(a.point_, b.point_); }

  bool Environment::IsVisible(const SpaceTimePoint &point_one, const SpaceTimePoint &point_two) { return IsVisibleRayCast(point_one, point_two); }

  bool Environment::IsVisibleRayCast(const SpaceTimePoint &point_one, const SpaceTimePoint &point_two)
  {

    // Skip the search if the two points are close (assumes the points themselves are not in collision)
    // if (dynamic_obstacles_.size() > 0 && RosTools::dist(point_one.Pos(), point_two.Pos()) < 2. * dynamic_obstacles_[0].radius_)
    //   return true;

    /** @note raycast implementation: scales with horizon length */
    /*https: // math.stackexchange.com/questions/2213165/find-shortest-distance-between-lines-in-3d (second solution )*/
    Eigen::Vector3d a, b, c, d, e;
    double A;
    double dist;

    a = point_one;
    b = point_two - point_one;

    for (auto &obstacle : dynamic_obstacles_)
    {
      if (obstacle.positions_.size() < 2)
        continue;
      for (int k = 0; k < Config::N; k++) // For constant velocity, only one line segment
      {
        c = Eigen::Vector3d(obstacle.positions_[k](0), obstacle.positions_[k](1), k);
        d = Eigen::Vector3d(obstacle.positions_[k + 1](0), obstacle.positions_[k + 1](1), k + 1) - c;
        // c = Eigen::Vector3d(obstacle.positions_[0](0), obstacle.positions_[0](1), 0);
        // d = Eigen::Vector3d(obstacle.positions_.back()(0), obstacle.positions_.back()(1), Config::N) - c;

        e = a - c;

        A = -(b.dot(b) * d.dot(d) - std::pow(b.dot(d), 2.));

        double s = (-(b.dot(b)) * (d.dot(e)) + (b.dot(e)) * (d.dot(b))) / A;
        double t = ((d.dot(d)) * (b.dot(e)) - (d.dot(e)) * (d.dot(b))) / A;

        s = std::max(0., std::min(s, 1.));
        t = std::max(0., std::min(t, 1.));

        // std::cout << "s: " << s << ", t: " << t << std::endl;
        dist = (e + b * t - d * s).norm();

        // std::cout << "distance: " << dist << std::endl;

        if (dist < obstacle.radius_)
          return false;
      }
    }

    return true;
  }

  /** Spawn a line between a and b, take steps along the line, discretize the time axis and validate collisions */
  bool Environment::IsVisibleRaySampling(const SpaceTimePoint &point_one, const SpaceTimePoint &point_two)
  {
    /** @note stepwise implementation: scales with space which is suboptimal */
    double step_size = 0.1;

    Eigen::Vector3d unit_vec = Eigen::Vector3d(point_two - point_one).normalized();        // Line ab
    int num_steps = std::floor(Eigen::Vector3d(point_two - point_one).norm() / step_size); // Step size

    Eigen::Vector3d cur_vec = point_one;

    bool result = true;
    // #pragma omp parallel for num_threads(8)
    for (int i = 1; i < num_steps; i++) // Check collision for each step in the direction a->b
    {
      if (InCollision(SpaceTimePoint(cur_vec + unit_vec * (float)i * step_size)))
      {
        // result = false;
        return false;
      }
    }
    return result;
  }

  void Environment::ProjectToFreeSpace(Eigen::Vector2d &point, int k, double with_margin)
  {

    // Projecting in 2D from obstacles
    for (auto &obstacle : dynamic_obstacles_)
    {
      double dist = (obstacle.positions_[k] - point).norm();
      if (dist < obstacle.radius_ * (1. + with_margin))
        point = obstacle.positions_[k] + (point - obstacle.positions_[k]).normalized() * (obstacle.radius_ * (1. + with_margin));
    }

    for (auto &halfspace : static_obstacles_)
    {
      if (halfspace.A_.transpose() * point > halfspace.b_)
      {
        // Ax > b -> move x by b - Ax in direction of A (plus the margin)
        // point += halfspace.A_ * (-halfspace.b_ + halfspace.A_.transpose() * point + with_margin);
      }
    }
  }

  void Environment::ProjectToFreeSpace(SpaceTimePoint &point, double with_margin)
  {

    Eigen::Vector2d new_pos = point.Pos();
    ProjectToFreeSpace(new_pos, point.Time(), with_margin);

    point.SetPos(new_pos);
  }

  /** Deprecated */
  bool Environment::IsVisible2D(const SpaceTimePoint &point_one, const SpaceTimePoint &point_two)
  {

    int k = point_one.Time();
    double hit_distance = 1.e9;

    const Eigen::Vector2d &a = point_one.Pos();
    const Eigen::Vector2d &b = point_two.Pos();

    double A, B, C, D;
    double lambda;

    for (auto &obstacle : dynamic_obstacles_)
    {
      const Eigen::Vector2d &c = obstacle.positions_[k]; // Circle origin

      // Intersect a line and a circle
      A = (a - b).dot(a - b);
      B = 2. * a.dot(b) + 2. * (b - a).dot(c) - 2. * b.dot(b);
      C = (b - c).dot(b - c) - obstacle.radius_ * obstacle.radius_;
      D = B * B - 4. * A * C;

      if (D >= 0) // If there are intersections
      {
        lambda = (-B - sqrt(D)) / (2 * A);

        // Check if there is an intersection before the end of the line
        if (lambda > 0. && lambda < 1.)
        {
          hit_distance = std::min(hit_distance, (lambda * b + (1. - lambda) * a - a).norm()); // Distance of line segment
          return false;                                                                       // Visibility is broken!
        }

        lambda = (-B + sqrt(D)) / (2 * A);

        // Check if there is an intersection before the end of the line
        if (lambda > 0. && lambda < 1.)
        {
          hit_distance = std::min(hit_distance, (lambda * b + (1. - lambda) * a - a).norm()); // Distance of line segment
          return false;                                                                       // Visibility is broken!
        }
      }
    }

    return true;
  }

  // GRIDDED ENVIRONMENT
  void GriddedEnvironment::Init() { grid_.SetDimensions(50, 50); }

  void GriddedEnvironment::ObstacleGrid::SetDimensions(double length, double width, double obstacle_radius)
  {
    grid_.resize(Config::N + 1); // + 1 to accomodate for the initial state

    // If the obstacle radius is larger, we should use a wider grid
    grid_size_ = 0.5;
    if (obstacle_radius > 0.5)
      grid_size_ = obstacle_radius;

    int new_size_x = std::ceil(length / grid_size_);
    int new_size_y = std::ceil(width / grid_size_);

    if (size_x_ == new_size_x && size_y_ == new_size_y) // Do not allocate if the size remained the same
      return;

    size_x_ = new_size_x;
    size_y_ = new_size_y;

#pragma omp parallel for num_threads(8)
    for (size_t k = 0; k < grid_.size(); k++)
    {
      // Initialize
      grid_[k].resize(size_x_);

      for (size_t i_x = 0; i_x < grid_[k].size(); i_x++)
      {
        grid_[k][i_x].resize(size_y_);
        for (size_t i_y = 0; i_y < grid_[k][i_x].size(); i_y++)
          grid_[k][i_x][i_y].reserve(2);
      }
    }
  }

  void GriddedEnvironment::ObstacleGrid::SetOrigin(const Eigen::Vector2d &pos)
  {
    // Place the grid with the robot in the center
    origin_ = Eigen::Vector2d(pos(0) - (double)size_x_ * grid_size_ / 2., pos(1) - (double)size_y_ * grid_size_ / 2.);
  }

  void GriddedEnvironment::ObstacleGrid::Clear()
  {
#pragma omp parallel for num_threads(8)
    for (size_t k = 0; k < grid_.size(); k++)
    {
      for (size_t i_x = 0; i_x < grid_[k].size(); i_x++)
      {
        for (size_t i_y = 0; i_y < grid_[k][i_x].size(); i_y++)
          grid_[k][i_x][i_y].clear();
      }
    }
  }

  void GriddedEnvironment::ObstacleGrid::InsertObstacle(int k, const SingleObstacle &obstacle)
  {
    int i_x, i_y;
    GetGridIndices(obstacle.position, i_x, i_y);

    // Add the obstacle in all neighbouring cells
#pragma omp parallel for num_threads(8)
    for (int i = -1; i <= 1; i++)
    {
      for (int j = -1; j <= 1; j++)
      {
        int this_i_x = i_x + i;
        int this_i_y = i_y + j;

        // If the grid cell exists, add the obstacle
        if (!OutOfGridBounds(this_i_x, this_i_y))
          grid_[k][this_i_x][this_i_y].emplace_back(obstacle);
      }
    }
  }

  std::vector<GriddedEnvironment::SingleObstacle> &GriddedEnvironment::ObstacleGrid::ObstaclesAt(const SpaceTimePoint &point)
  {
    if (grid_.size() == 0)
      return empty_obstacle_list_;

    int i_x, i_y;
    GetGridIndices(point.Pos(), i_x, i_y);

    if (OutOfGridBounds(i_x, i_y))
      return empty_obstacle_list_;

    return grid_[std::round(point.Time())][i_x][i_y]; // Return the list of obstacles at this grid index
  }

  void GriddedEnvironment::ObstacleGrid::Print()
  {
    std::cout << "Grid: [" << grid_.size() << " x " << grid_[0].size() << " x " << grid_[0][0].size() << "]" << std::endl;
  }

  bool GriddedEnvironment::ObstacleGrid::OutOfGridBounds(int i_x, int i_y)
  {
    return i_x < 0 || i_x >= size_x_ || i_y < 0 || i_y >= size_y_; // If it is outside the grid
  }

  void GriddedEnvironment::ObstacleGrid::GetGridIndices(const Eigen::Vector2d &pos, int &i_x_out, int &i_y_out)
  {
    // Find the grid index
    i_x_out = std::floor((pos(0) - origin_(0)) / grid_size_);
    i_y_out = std::floor((pos(1) - origin_(1)) / grid_size_);
  }

  bool GriddedEnvironment::InCollision(const SpaceTimePoint &point, double with_margin)
  {
    const std::vector<SingleObstacle> &cur_grid_obstacles = grid_.ObstaclesAt(point);
    // std::cout << "number of obstacles there: " << cur_grid_obstacles.size() << std::endl;

    for (auto &obstacle : cur_grid_obstacles)
    {
      if (RosTools::distance(obstacle.position, point.Pos()) <
          obstacle.radius + with_margin) // Note that the obstacle positions at k = 0 is the initial state
        return true;
    }

    for (auto &halfspace : static_obstacles_)
    {
      if (halfspace.A_.transpose() * point.Pos() > halfspace.b_)
        return true;
    }

    return false;
  }

  void GriddedEnvironment::LoadObstacles(const std::vector<Obstacle> &dynamic_obstacles, const std::vector<Halfspace> &static_obstacles)
  {
    Environment::LoadObstacles(dynamic_obstacles, static_obstacles);

    if (dynamic_obstacles_.size() > 0)
      grid_.SetDimensions(50, 50, dynamic_obstacles_[0].radius_);
    else
      grid_.SetDimensions(50, 50, 0.5);
    grid_.Clear();

    for (auto &obstacle : dynamic_obstacles_)
    {
      PRM_LOG("Obstacle path len: " << obstacle.positions_.size());
      for (int k = 0; k < Config::N + 1; k++)
        grid_.InsertObstacle((int)k, SingleObstacle(obstacle.positions_[k], obstacle.radius_));
    }
  }

} // namespace Homotopy