#include "guidance_planner/cubic_spline.h"

using namespace GuidancePlanner;

CubicSpline3D::CubicSpline3D(const GeometricPath &path, Config *config, const Eigen::Vector2d &current_velocity)
    : id_(path.association_.id_), config_(config), current_velocity_(current_velocity)
{
  // Convert the path in 3D to a 2D trajectory
  ConvertToTrajectory(path);

  acceleration_weights_computed_ = false;

  longitudinal_acceleration_weight_ = 0.;
  lateral_acceleration_weight_ = 0.;

  std::vector<double> x_points, y_points;
  control_points_.GetX(x_points); // Get the vector of x including padding
  control_points_.GetY(y_points); // Get the vector of y including padding

  // Set the velocity as a boundary condition of the spline
  // x_.set_boundary(tk::spline::bd_type::first_deriv, previous_velocity_(0), tk::spline::bd_type::second_deriv, 0.);
  // y_.set_boundary(tk::spline::bd_type::first_deriv, previous_velocity_(1), tk::spline::bd_type::second_deriv, 0.);
  x_.set_boundary(tk::spline::bd_type::first_deriv, current_velocity_(0), tk::spline::bd_type::second_deriv, 0.);
  y_.set_boundary(tk::spline::bd_type::first_deriv, current_velocity_(1), tk::spline::bd_type::second_deriv, 0.);

  control_points_.GetT(t_padded_); // Get the vector of times including padding

  x_.set_points(t_padded_, x_points);
  y_.set_points(t_padded_, y_points);

  // Remove the added previous point, since it is not part of our new trajectory (but was necessary for the boundary conditions)
  // x_.RemoveStart();
  // y_.RemoveStart();
  // t_padded_.erase(t_padded_.begin());

  trajectory_spline_.reset(new RosTools::CubicSpline2D<tk::spline>(x_, y_));

  // DebugPrintControlPoints();

  ComputePath();

  PRM_LOG("CubicSpline created");
}

void CubicSpline3D::ConvertToTrajectory(const GeometricPath &path)
{

  // Initialize control points from [1 - N-1]
  control_points_ = ControlPoints(config_->num_points_ - 2); // First point is the start / last the end // Config::N - 1);
  // control_points_ = ControlPoints(path.nodes_.size() - 2); // config_->num_points_ - 2); // First point is the start / last the end //
  // Config::N - 1);

  // Add the start (Point at k = 0!)
  control_points_.PadStart(path(0.));

  Eigen::ArrayXd sampled_k = Eigen::ArrayXd::LinSpaced(config_->num_points_, 0., Config::N);

  // Eigen::ArrayXd sampled_k = Eigen::ArrayXd::Zero(path.nodes_.size());
  // for (int k = 0; k < sampled_k.rows(); k++)
  //     sampled_k(k) = path.nodes_[k]->point_.Time();

  /** @note We need to find throughout the spline, where the integer "k"s are */
  double last_k = -0.1;
  for (int k_idx = 1; k_idx < sampled_k.size() - 1; k_idx++) // Then add from the first until the last sampled point (which exclude start and end)
  {
    double k = sampled_k[k_idx];

    // A bisection finds the point where the 3rd coordinate of this spline is equal to "k".
    double s_cur = RosTools::Bisection(
        last_k, 1., [&](double s)
        { return path(s)(2) - k; },
        1e-3);

    control_points_.AddPoint(path(s_cur));

    last_k = s_cur; // Limit the search to [last_k, 1]

    // control_points_.PrintLast();
  }

  // Add the end
  control_points_.PadEnd(path(1.));

  // Time difference per two control points
  delta_t_ = control_points_.GetDeltaT(); // / (sampled_k(1) - sampled_k(0));

  initial_control_points_ = control_points_;

  PRM_LOG("Converted Path into Trajectory");
}

void CubicSpline3D::PadStart(const SpaceTimePoint &start_padding, const Eigen::Vector2d &previous_velocity)
{
  PRM_LOG("Adding padding at the start of the BSpline");

  previous_velocity_ = previous_velocity;

  control_points_.PadStart(start_padding);
}

void CubicSpline3D::Optimize(const std::vector<Obstacle> &obstacles)
{
  PRM_LOG("BSpline::Optimize()");

  // Compute the velocity control points

  for (int repeat_id = 0; repeat_id < config_->repeat_times_; repeat_id++)
  {

    // CLOSE TO GEOMETRIC PATH //
    PRM_LOG("BSpline Optimize - Adding cost to geometric path");

    // Vector of G (points on the original path)
    Eigen::VectorXd f_g = Eigen::Map<Eigen::VectorXd>(initial_control_points_.points_.data(),
                                                      initial_control_points_.points_.cols() * initial_control_points_.points_.rows());

    // Simply identity weights
    Eigen::MatrixXd H_g =
        config_->geometric_weight_ * 2. * Eigen::MatrixXd::Identity(control_points_.NumPoints() * 2, control_points_.NumPoints() * 2);
    f_g = config_->geometric_weight_ * -2 * f_g;

    // SMOOTHNESS //
    PRM_LOG("BSpline Optimize - Adding smoothness cost");

    Eigen::MatrixXd H_s = Eigen::MatrixXd::Zero(2 * control_points_.NumPoints(), 2 * control_points_.NumPoints());
    Eigen::VectorXd f_s;

    bool one_padding = true;
    Eigen::MatrixXd M1, M3;

    if (one_padding)
    {
      M1 = Eigen::MatrixXd::Zero(2 * 1, control_points_.NumPoints() * 2); // Start, 2 = num of not optimized points at the sides
      M3 = Eigen::MatrixXd::Zero(2 * 1, control_points_.NumPoints() * 2); // End
    }
    else
    {
      M1 = Eigen::MatrixXd::Zero(2 * 2, control_points_.NumPoints() * 2); // Start, 2 = num of not optimized points at the sides
      M3 = Eigen::MatrixXd::Zero(2 * 2, control_points_.NumPoints() * 2); // End
    }

    for (int diag_id = 0; diag_id < control_points_.NumPoints() && diag_id < 3; diag_id++)
    {
      double val;

      switch (diag_id)
      {
      case 0:
        val = 6;

        break;
      case 1:
        val = -4;
        break;
      case 2:
        val = 1;
        break;
      }
      // Set each value along the selected diagonal
      for (int j = 0; j < H_s.rows() - diag_id * 2; j++)
      {
        if (one_padding && diag_id == 0 && (j <= 1 || j >= H_s.rows() - diag_id * 2 - 2)) // Extension for one control point on the side

          H_s(j + diag_id * 2, j) = val - 1; // Set the lower diagonal
        else
          H_s(j + diag_id * 2, j) = val; // Set the lower diagonal

        if (diag_id > 0)                 // For diag_id == 0, lower = upper
          H_s(j, j + diag_id * 2) = val; // Set the upper diagonal
      }

      if (diag_id >= 2)
        continue;

      double f_val;
      if (one_padding)
      {
        switch (diag_id)
        {
        case 0:
          f_val = -2;
          break;
        case 1:
          f_val = 1;
          break;
        }
        for (int j = 0; j < 2; j++)
        {
          M1(j, j + 2 * diag_id) = f_val;
          M3(j, M3.cols() - diag_id * 2 - 2 + j) = f_val;
        }
      }
      else
      {
        switch (diag_id)
        {
        case 0:
          f_val = 1;
          break;
        case 1:
          f_val = -4;
          break;
        }
        for (int j = 0; j < M1.rows() - diag_id * 2; j++)
        {
          M1(j + diag_id * 2, j) = f_val;
          M3(j, M3.cols() - 2 * 2 + j + diag_id * 2) = f_val;
        }
      }
    }

    // std::cout << H_s << std::endl;
    // std::cout << M1 << std::endl;
    // std::cout << M3 << std::endl;

    // Construct f from the padded points on both sides of the spline
    Eigen::VectorXd start, end;
    control_points_.GetStartVectorized(start);
    control_points_.GetEndVectorized(end); /** @todo End is incorrect now (should be only 1) - Although it is okay */

    f_s = start.transpose() * M1;
    f_s += end.transpose() * M3; // = half, but we solve for xHx + 2fx

    // Add this cost
    H_g += config_->smoothness_weight_ * H_s;
    f_g += config_->smoothness_weight_ * f_s;

    // OBSTACLES REPULSIVE//
    PRM_LOG("BSpline Optimize - Adding obstacle avoidance cost");

    // Distance = Ax - b, e^(-x) taylor expansion 2nd degree -> quadratic cost
    // For each control point
    for (int i = 0; i < control_points_.NumPoints(); i++)
    {
      // Check for all obstacles
      for (auto &obstacle : obstacles)
      {
        // Index 1 - N
        Eigen::Vector2d obstacle_pos =
            obstacle.positions_[std::round(control_points_.GetTime(i) / Config::DT)]; // std::round(control_points_(2, i))];
        Eigen::Vector2d control_pos = control_points_.GetPoint(i);

        // Compute the distance to the obstacle
        double dist = RosTools::dist(obstacle_pos, control_pos);

        // Only add constraints for obstacles that are close
        if (dist > obstacle.radius_ * 1.5)
          continue;

        Eigen::Vector2d A_i = -(obstacle_pos - control_pos).normalized();

        Eigen::MatrixXd H_i = A_i * A_i.transpose() / 2.;

        double b_i = obstacle.radius_;

        Eigen::VectorXd f_i = -(1 + b_i) * A_i.transpose() - 2 * obstacle_pos.transpose() * H_i.transpose(); /*A_i * A_i.transpose()*/
                                                                                                             // Obstacle f is missing

        H_g.block(i * 2, i * 2, 2, 2) = H_g.block(i * 2, i * 2, 2, 2) + config_->collision_weight_ * H_i;
        f_g.block(i * 2, 0, 2, 1) = f_g.block(i * 2, 0, 2, 1) + config_->collision_weight_ * f_i;
      }
    }

    // VELOCITY TRACKING
    // Idea here is to first compute the velocity control points of the current spline
    // Then we penalize distance to this spline, which should try to track the velocity
    // Note that otherwise, the velocity penalty is 4th order rather than quadratic
    Eigen::MatrixXd velocity_spline_control_points_ = ComputeVelocitySplinePoints();
    Eigen::VectorXd f_v = Eigen::Map<Eigen::VectorXd>(velocity_spline_control_points_.data(),
                                                      velocity_spline_control_points_.cols() * velocity_spline_control_points_.rows());

    // Simply identity weights
    Eigen::MatrixXd H_v =
        config_->velocity_tracking_ * 2. * Eigen::MatrixXd::Identity(control_points_.NumPoints() * 2, control_points_.NumPoints() * 2);
    f_v = config_->velocity_tracking_ * -2 * f_v;

    H_g += H_v;
    f_g += f_v;

    // SOLVE //
    // Unconstrained QP analytic solution
    Eigen::VectorXd new_points_vectorized = -H_g.inverse() * f_g;

    // Convert vector back to matrix format
    for (int i = 0; i < control_points_.NumPoints(); i++)
      control_points_.SetPoint(i, new_points_vectorized.block(i * 2, 0, 2, 1));
  }

  // POST-CHECK, PROJECT POINTS OUTSIDE OF OBSTACLES //
  if (config_->project_from_obstacles_)
  {
    for (int i = 0; i < control_points_.NumPoints(); i++)
    {
      Eigen::Vector2d cur_point = control_points_.GetPoint(i);
      int k = std::round(control_points_.GetTime(i) / Config::DT);

      for (auto &obstacle : obstacles)
      {
        Eigen::Vector2d diff = cur_point - obstacle.positions_[k];
        if (diff.norm() < obstacle.radius_) // If in the obstacle
        {
          cur_point = obstacle.positions_[k] + diff.normalized() * obstacle.radius_; // Project away
          control_points_.SetPoint(i, cur_point);                                    // Load
        }
      }
    }
  }

  // POST-PROCESSING OF THE OPTIMIZED SPLINE //
  // Now that we have control points of the spline, construct it
  std::vector<double> x_points, y_points;
  control_points_.GetX(x_points); // Get the vector of x including padding
  control_points_.GetY(y_points); // Get the vector of y including padding

  // Set the velocity as a boundary condition of the spline
  // x_.set_boundary(tk::spline::bd_type::first_deriv, previous_velocity_(0), tk::spline::bd_type::second_deriv, 0.);
  // y_.set_boundary(tk::spline::bd_type::first_deriv, previous_velocity_(1), tk::spline::bd_type::second_deriv, 0.);
  x_.set_boundary(tk::spline::bd_type::first_deriv, current_velocity_(0), tk::spline::bd_type::second_deriv, 0.);
  y_.set_boundary(tk::spline::bd_type::first_deriv, current_velocity_(1), tk::spline::bd_type::second_deriv, 0.);

  control_points_.GetT(t_padded_); // Get the vector of times including padding

  x_.set_points(t_padded_, x_points);
  y_.set_points(t_padded_, y_points);

  // Remove the added previous point, since it is not part of our new trajectory (but was necessary for the boundary conditions)
  // x_.RemoveStart();
  // y_.RemoveStart();
  // t_padded_.erase(t_padded_.begin());

  trajectory_spline_.reset(new RosTools::CubicSpline2D<tk::spline>(x_, y_));

  // DebugPrintControlPoints();

  ComputePath();
}

Eigen::MatrixXd CubicSpline3D::ComputeVelocitySplinePoints()
{
  velocity_control_points_ = Eigen::MatrixXd(2, control_points_.size() - 1);

  // In the velocity spline, we do not care about the first padding (the previous state)
  // First compute all velocity control points (1 - N-2)
  for (int i = 0; i < control_points_.size() - 1; i++)
    velocity_control_points_.col(i) = 1. / delta_t_ * (control_points_.GetPaddedPoint(i + 1) - control_points_.GetPaddedPoint(i));

  // Normalize the velocities and multiply them with the reference velocity
  for (int i = 0; i < velocity_control_points_.cols(); i++)
    velocity_control_points_.col(i) =
        velocity_control_points_.col(i).normalized().array() * Config::reference_velocity_; /** @todo input vref */

  // SMOOTHNESS //
  // if (config_->smoothen_velocity_)
  // {
  //   Eigen::MatrixXd H_s = Eigen::MatrixXd::Zero(2 * velocity_control_points_.cols(), 2 * velocity_control_points_.cols());
  //   Eigen::VectorXd f_s;

  //   Eigen::MatrixXd M1 = Eigen::MatrixXd::Zero(2 * 2, velocity_control_points_.cols() * 2); // Start, 2 = num of not optimized points at the sides
  //   Eigen::MatrixXd M3 = Eigen::MatrixXd::Zero(2 * 2, velocity_control_points_.cols() * 2); // End
  //   for (int diag_id = 0; diag_id < velocity_control_points_.cols() && diag_id < 3; diag_id++)
  //   {
  //     double val;
  //     switch (diag_id)
  //     {
  //     case 0:
  //       val = 6;
  //       break;
  //     case 1:
  //       val = -4;
  //       break;
  //     case 2:
  //       val = 1;
  //       break;
  //     }

  //     // Set each value along the selected diagonal
  //     for (int j = 0; j < H_s.rows() - diag_id * 2; j++)
  //     {
  //       H_s(j + diag_id * 2, j) = val;   // Set the lower diagonal
  //       if (diag_id > 0)                 // For diag_id == 0, lower = upper
  //         H_s(j, j + diag_id * 2) = val; // Set the upper diagonal
  //     }

  //     if (diag_id >= 2)
  //       continue;

  //     double f_val;
  //     switch (diag_id)
  //     {
  //     case 0:
  //       f_val = 1;
  //       break;
  //     case 1:
  //       f_val = -4;
  //       break;
  //     }

  //     for (int j = 0; j < M1.rows() - diag_id * 2; j++)
  //     {
  //       M1(j + diag_id * 2, j) = f_val;
  //       M3(j, M3.cols() - 2 * 2 + j + diag_id * 2) = f_val;
  //     }
  //   }

  //   // Construct f from the padded points on both sides of the spline
  //   Eigen::VectorXd start, end;

  //   // First velocity, twice
  //   start = Eigen::VectorXd(4);
  //   start << previous_velocity_(0), previous_velocity_(1), previous_velocity_(0),
  //       previous_velocity_(1); // velocity_control_points_(0, 0), velocity_control_points_(1, 0);

  //   // Last velocity, twice
  //   end = Eigen::VectorXd(4);
  //   end << velocity_control_points_.rightCols(1)(0), velocity_control_points_.rightCols(1)(1), velocity_control_points_.rightCols(1)(0),
  //       velocity_control_points_.rightCols(1)(1);

  //   f_s = start.transpose() * M1;
  //   f_s += end.transpose() * M3;

  //   // SOLVE //
  //   // Unconstrained QP analytic solution
  //   Eigen::VectorXd new_points_vectorized = -H_s.inverse() * f_s;
  //   // Convert vector back to matrix format
  //   for (int i = 0; i < velocity_control_points_.cols(); i++)
  //     velocity_control_points_.block<2, 1>(0, i) = new_points_vectorized.block(i * 2, 0, 2, 1);
  // }

  // DONE //
  // We construct a trajectory that follows the reference velocity
  Eigen::MatrixXd ideal_velocity_spline(2, control_points_.NumPoints());

  // We construct this trajectory by adding the velocities one by one
  // We do not care here about the previous position of the vehicle
  // Q_1 = Q_0 + DT * V_0 // shouldn't this be "GetPaddedPoint(0)?"
  ideal_velocity_spline.col(0) = control_points_.GetPaddedPoint(1) + delta_t_ * velocity_control_points_.col(0); // Start at the current position
  for (int i = 0; i < control_points_.NumPoints() - 1; i++)
    ideal_velocity_spline.col(i + 1) = ideal_velocity_spline.col(i) + delta_t_ * velocity_control_points_.col(i + 1);

  return ideal_velocity_spline;
  /** @todo Consider smoothness optimization here */
}

void CubicSpline3D::ComputePath()
{

  // Vector with distances
  std::vector<double> d_points;
  d_points.push_back(0.); // Distance at the start

  /** @note First we find, at the defined points, their distances */
  double length = 0.;

  for (int i = 0; i < control_points_.NumPoints() + 1; i++) // Compute distances from k=1 until k=N (last point is not optimized)
  {
    Eigen::VectorXd sampled_time = Eigen::VectorXd::LinSpaced(10, t_padded_[i], t_padded_[i + 1]);
    Eigen::Vector2d cur_point, prev_point;

    for (int j = 0; j < sampled_time.rows(); j++)
    {
      // std::cout << "at t = " << sampled_time(j) << ": " << x_(sampled_time(j)) << ", " << y_(sampled_time(j)) << std::endl;
      cur_point(0) = x_(sampled_time(j)); // Evaluate the spline at the sampled s
      cur_point(1) = y_(sampled_time(j)); // Evaluate the spline at the sampled s

      if (j > 0)
        length += (cur_point - prev_point).norm();

      prev_point = cur_point;
    }
    // Add the length up until this point
    d_points.push_back(length);
  }

  // First we convert the spline range from 0-1, to 0-S, with S the maximum distance and s in R
  tk::spline cubic_spline_x, cubic_spline_y;

  // Boundary condition (set direction of the spline from the previous state)
  // We constraint here dx/ds = dx/dt dt/ds, where dt/ds is 1 / (ds/dt) = ||v||

  // double dt_ds = 1. / previous_velocity_.norm();
  // if (previous_velocity_.norm() > 1e-8)
  // {
  //     cubic_spline_x.set_boundary(tk::spline::bd_type::first_deriv, previous_velocity_(0) * dt_ds, tk::spline::bd_type::second_deriv, 0.);
  //     cubic_spline_y.set_boundary(tk::spline::bd_type::first_deriv, previous_velocity_(1) * dt_ds, tk::spline::bd_type::second_deriv, 0.);
  // }
  // else // If the velocity is 0, add it explicitly
  // {
  //     cubic_spline_x.set_boundary(tk::spline::bd_type::first_deriv, 0., tk::spline::bd_type::second_deriv, 0.);
  //     cubic_spline_y.set_boundary(tk::spline::bd_type::first_deriv, 0., tk::spline::bd_type::second_deriv, 0.);
  // }
  // std::cout << d_points.size() << ", " << x_.m_y_.size() << ", " << y_.m_y_.size() << std::endl;

  /*std::cout << "d: ";
  for (auto &point : d_points)
      std::cout << point << ", ";

  std::cout << "\b\b\n";

  std::cout << "x: ";
  for (auto &point : x_.m_y_)
      std::cout << point << ", ";

  std::cout << "\b\b\n";
  std::cout << "y: ";
  for (auto &point : y_.m_y_)
      std::cout << point << ", ";

  std::cout << "\b\b\n";*/

  cubic_spline_x.set_points(d_points, x_.m_y_); // Create new splines with the old x/y, but over the computed distances
  cubic_spline_y.set_points(d_points, y_.m_y_);

  path_spline_.reset(new RosTools::CubicSpline2D<tk::spline>(cubic_spline_x, cubic_spline_y));

  PRM_LOG("Path Spline Computed");
}

RosTools::CubicSpline2D<tk::spline> &CubicSpline3D::GetPath() { return *path_spline_; }

RosTools::CubicSpline2D<tk::spline> &CubicSpline3D::GetTrajectory() const { return *trajectory_spline_; }

std::vector<Eigen::Vector3d> &CubicSpline3D::GetSamples()
{

  sampled_points_.clear();

  Eigen::VectorXd sampled_t = Eigen::VectorXd::LinSpaced(100, 0., trajectory_spline_->GetSplineLength());

  for (int i = 0; i < sampled_t.size(); i++)
  {
    double t = sampled_t(i);
    Eigen::Vector2d cur_point = trajectory_spline_->GetPoint(t);
    sampled_points_.emplace_back(cur_point(0), cur_point(1), t);
  }
  return sampled_points_;
}

double CubicSpline3D::WeightPathLength() { return path_spline_->GetSplineLength(); }

double CubicSpline3D::WeightVelocity()
{
  double result = 0.;

  Eigen::ArrayXd t_sampled = Eigen::ArrayXd::LinSpaced(100, 0., trajectory_spline_->GetSplineLength());

  for (int i = 0; i < t_sampled.rows(); i++)
  {
    double cur_velocity = trajectory_spline_->GetVelocity(t_sampled[i]).norm();

    result += (Config::reference_velocity_ - cur_velocity) *
              (Config::reference_velocity_ - cur_velocity); // Quadratic error w.r.t. the reference
  }

  // Average the error over the number of evaluations
  result /= (double)t_sampled.rows();

  return result;
}

double CubicSpline3D::WeightLongitudinalAcceleration()
{
  if (!acceleration_weights_computed_)
    ComputeAccelerationWeights();

  return longitudinal_acceleration_weight_;
}

double CubicSpline3D::WeightLateralAcceleration()
{
  // if (!acceleration_weights_computed_)
  // ComputeAccelerationWeights();

  return 0.; // lateral_acceleration_weight_;
}

void CubicSpline3D::ComputeAccelerationWeights()
{

  longitudinal_acceleration_weight_ = 0.;
  lateral_acceleration_weight_ = 0.;

  Eigen::ArrayXd t_sampled = Eigen::ArrayXd::LinSpaced(100, 0., trajectory_spline_->GetSplineLength());

  for (int i = 0; i < t_sampled.rows(); i++)
  {
    // Determine the orientation along the trajectory
    // Eigen::Vector2d cur_vel = trajectory_spline_->GetVelocity(t_sampled[i]);
    // Eigen::MatrixXd R = RosTools::rotationMatrixFromHeading(-std::atan2(cur_vel(1), cur_vel(0)));

    // Get the acceleration in longitudinal / lateral direction
    // double cur_accel_long = (R * trajectory_spline_->GetAcceleration(t_sampled[i]))(0);
    // longitudinal_acceleration_weight_ += cur_accel_long * cur_accel_long;

    // double cur_accel_lat = (R * trajectory_spline_->GetAcceleration(t_sampled[i]))(1);
    // lateral_acceleration_weight_ += cur_accel_lat * cur_accel_lat;

    /** @note Just the norm of the acceleration instead of separate terms */
    longitudinal_acceleration_weight_ += trajectory_spline_->GetAcceleration(t_sampled[i]).norm() * std::pow(0.95, i);
  }

  longitudinal_acceleration_weight_ /= (double)t_sampled.rows();
  // lateral_acceleration_weight_ /= (double)t_sampled.rows();

  acceleration_weights_computed_ = true;
}

void CubicSpline3D::Visualize(RosTools::ROSMarkerPublisher *ros_visuals)
{

  RosTools::ROSPointMarker &cube = ros_visuals->getNewPointMarker("CUBE");
  cube.setScale(0.15, 0.15, 0.15);
  cube.setColor(0., 0., 0.);

  RosTools::ROSLine &line = ros_visuals->getNewLine();
  line.setScale(0.25, 0.25, 0.25);
  line.setColor(1., 0., 0.);

  // The final control points
  for (int i = 0; i < control_points_.NumPoints(); i++)
  {
    cube.addPointMarker(Eigen::Vector3d(control_points_.GetPoint(i)(0), control_points_.GetPoint(i)(1), control_points_.GetTime(i)));
  }

  // Plot the initial control points
  cube.setColor(0., 1., 0., 0.6);
  cube.setScale(0.2, 0.2, 0.2);
  for (int i = 0; i < initial_control_points_.NumPoints(); i++)
  {
    cube.addPointMarker(
        Eigen::Vector3d(initial_control_points_.GetPoint(i)(0), initial_control_points_.GetPoint(i)(1), initial_control_points_.GetTime(i)));
  }

  // Plot paddings
  cube.setColor(1., 0., 0.);
  for (int i = 0; i < 1; i++)
  {
    // if (i == 1)
    // {
    //     cube.setColor(1., 0., 0.);
    //     cube.setScale(0.2, 0.2, 0.2);
    // }
    // else if (i == 0)
    // {
    cube.setColor(1., 0., 0.4, 0.4);
    cube.setScale(0.15, 0.15, 0.15);
    line.addLine(Eigen::Vector3d(control_points_.GetPaddedPoint(i)(0), control_points_.GetPaddedPoint(i)(1), control_points_.GetPaddedTime(i)),
                 Eigen::Vector3d(control_points_.GetPaddedPoint(i)(0) + current_velocity_(0),
                                 control_points_.GetPaddedPoint(i)(1) + current_velocity_(1), control_points_.GetPaddedTime(i)));
    // }
    cube.addPointMarker(
        Eigen::Vector3d(control_points_.GetPaddedPoint(i)(0), control_points_.GetPaddedPoint(i)(1), control_points_.GetPaddedTime(i)));
  }
}

void CubicSpline3D::DebugPrintControlPoints()
{

  PRM_LOG("==== CubicSpline3D (Initial Control Points) ====");
  for (int j = 0; j < 2; j++)
  {
    if (j == 0)
      std::cout << "x:\t";
    else
      std::cout << "y:\t";

    for (int i = 0; i < initial_control_points_.NumPoints(); i++)
      std::cout << initial_control_points_.GetPoint(i)(j) << ",\t";

    std::cout << "\b\b\n";
  }
  std::cout << std::endl;

  PRM_LOG("==== CubicSpline3D (Control Points) ====");
  for (int j = 0; j < 2; j++)
  {
    if (j == 0)
      std::cout << "x:\t";
    else
      std::cout << "y:\t";

    for (int i = 0; i < control_points_.NumPoints(); i++)
      std::cout << control_points_.GetPoint(i)(j) << ",\t";

    std::cout << "\b\b\n";
  }
  std::cout << std::endl;

  std::cout << "Time points:\n";
  for (auto &time_point : t_padded_)
  {
    std::cout << time_point << ", ";
  }
  std::cout << "\b\b\n";
}