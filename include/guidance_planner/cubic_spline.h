#ifndef __CUBIC_SPLINE_H__
#define __CUBIC_SPLINE_H__

#include <ros/ros.h>

#include <vector>
#include <Eigen/Dense>

#include <guidance_planner/paths.h>
#include <guidance_planner/types.h>

#include <lmpcc_tools/ros_visuals.h>

#include <third_party/spline.h>

namespace GuidancePlanner
{

    /** @brief Struct to simplify working with control points that are padded on both sides */
    struct ControlPoints
    {
        Eigen::MatrixXd points_;

    private:
        Eigen::MatrixXd start_;
        Eigen::MatrixXd end_;

        std::vector<double> t_start_, t_points_, t_end_;

        int num_points_;
        int cur_col;

    public:
        ControlPoints(int num_points_without_padding = 0)
        {
            num_points_ = num_points_without_padding;
            points_ = Eigen::MatrixXd(2, num_points_);
            t_points_.reserve(num_points_);
            cur_col = 0;
        }

        int size() const
        {
            return start_.cols() + points_.cols() + end_.cols();
        }

        int NumPoints() const
        {
            return points_.cols();
        }

        // Add an optimized point
        void AddPoint(const SpaceTimePoint &point)
        {
            points_.col(cur_col) = point.Pos();
            t_points_.push_back(point.Time() * HomotopyConfig::DT);
            cur_col++;
        }

        // Get an optimized point
        Eigen::Vector2d GetPoint(int i)
        {
            return points_.col(i);
        }

        // Get the time of an optimized point
        double GetTime(int i)
        {
            return t_points_[i];
        }

        // Get a point from the padded list of control points
        Eigen::Vector2d GetPaddedPoint(int i) const
        {
            if (i < start_.cols())
                return start_.col(i);
            else if (i < start_.cols() + points_.cols())
                return points_.col(i - start_.cols());
            else
                return end_.col(i - points_.cols() - start_.cols());
        }

        double GetPaddedTime(int i) const
        {
            if (i < start_.cols())
                return t_start_[i];
            else if (i < start_.cols() + points_.cols())
                return t_points_[i - start_.cols()];
            else
                return t_end_[i - points_.cols() - start_.cols()];
        }

        // Get a vector over the x of these control points
        void GetX(std::vector<double> &x_points) const
        {
            x_points.clear();
            for (int i = 0; i < size(); i++)
            {
                x_points.push_back(GetPaddedPoint(i)(0));
            }
        }

        // Get a vector over the y of these control points
        void GetY(std::vector<double> &y_points) const
        {
            y_points.clear();
            for (int i = 0; i < size(); i++)
            {
                y_points.push_back(GetPaddedPoint(i)(1));
            }
        }

        // Get a vector over the time of these control points
        void GetT(std::vector<double> &t_points) const
        {
            t_points = t_start_;
            t_points.insert(t_points.end(), t_points_.begin(), t_points_.end());
            t_points.insert(t_points.end(), t_end_.begin(), t_end_.end());
        }

        // Get the time step
        double GetDeltaT() const
        {
            // LMPCC_ASSERT(t_points_.size() > 1, "Delta T can only be computed if points are loaded");
            // return t_points_[1] - t_points_[0];
            return t_points_[0] - t_start_[0];
        }

        // Assign a value to an optimized point
        void SetPoint(int i, const Eigen::Vector2d &value)
        {
            points_.col(i) = value;
        }

        // Pad the start of the control points
        void PadStart(const SpaceTimePoint &start_padding)
        {
            LMPCC_ASSERT(t_start_.size() == 0, "Only one start padding is supported");

            start_ = start_padding.Pos();
            t_start_.push_back(start_padding.Time() * HomotopyConfig::DT);
        }

        // Pad the end of the control points
        void PadEnd(const SpaceTimePoint &end_padding)
        {
            LMPCC_ASSERT(t_end_.size() == 0, "Only one end padding is supported");

            end_ = end_padding.Pos();
            t_end_.push_back(end_padding.Time() * HomotopyConfig::DT);
        }

        void GetStartVectorized(Eigen::VectorXd &start)
        {
            start = Eigen::Vector2d(start_(0), start_(1));
        }

        void GetEndVectorized(Eigen::VectorXd &end)
        {
            end = Eigen::Vector2d(end_(0), end_(1));
        }

        void PrintLast()
        {
            std::cout << "(" << points_(0, cur_col - 1) << ", " << points_(1, cur_col - 1) << "), t = " << t_points_.back() << std::endl;
        }
    };

    class CubicSpline3D
    {

    public:
        int id_;
        double stored_cost_; // Needs to be set externally, but can then be used to easily sort splines

        /** @brief Initialize a Cubic Spline from an existing path (i.e., initial control points are on the path) */
        CubicSpline3D(const GeometricPath &path, HomotopyConfig *config, const Eigen::Vector2d &current_velocity);

        CubicSpline3D(const CubicSpline3D &other) = default;

        /** @brief Static function when an empty trajectory is needed */
        static CubicSpline3D &Empty(const Eigen::Vector2d &start, HomotopyConfig *config)
        {
            // Get some points close to the start
            static Node a(-1, {start(0), start(1), 0.}, NodeType::NONE);
            static Node mid(-1, {start(0) + 0.005, start(1), (double)HomotopyConfig::N / 2}, NodeType::NONE);
            static Node b(-1, {start(0) + 0.01, start(1), (double)HomotopyConfig::N}, NodeType::NONE); // Small forward deviation to make the path valid

            // Create a path
            std::vector<Node *> nodes;
            nodes.push_back(&a);
            nodes.push_back(&mid);
            nodes.push_back(&b);
            GeometricPath empty_path(nodes);

            // Define an empty trajectory
            static std::unique_ptr<CubicSpline3D> empty_trajectory;
            empty_trajectory.reset(new CubicSpline3D(empty_path, config, Eigen::Vector2d(0., 0.)));
            empty_trajectory->Optimize({});

            return *empty_trajectory;
        }

        void PadStart(const SpaceTimePoint &start_padding, const Eigen::Vector2d &previous_velocity);

        /** @brief Optimize the control points, except for the first and last point and the padding */
        void Optimize(const std::vector<Obstacle> &obstacles);

        /** @brief Convert this B-Spline to a cubic spline (tk::spline) to be used as reference path */
        CubicSpline2D<tk::spline> &GetPath();

        /** @brief Convert this spline to be parameterized in discrete time (i.e., its third dimension) */
        CubicSpline2D<tk::spline> &GetTrajectory() const;

        /** WEIGHTING Functions (map B-Spline to scalars) */
        /** @brief Return the path length of this spline */
        double WeightPathLength();

        /** @brief Return violations of the maximum velocity */
        double WeightVelocity();

        /** @brief Return a measure of the longitudinal acceleration */
        double WeightLongitudinalAcceleration();
        double longitudinal_acceleration_weight_;

        /** @brief Return a measure of the lateral acceleration */
        double WeightLateralAcceleration();
        double lateral_acceleration_weight_;

        std::vector<Eigen::Vector3d> &GetSamples();

        void Visualize(ROSMarkerPublisher *ros_visuals);

    private:
        // Spline in 3D
        tk::spline x_;
        tk::spline y_;
        tk::spline t_;

        HomotopyConfig *config_;
        int num_points_;

        Eigen::Vector2d previous_velocity_, current_velocity_;

        // The main control points
        ControlPoints control_points_, initial_control_points_;

        std::vector<double> t_padded_;
        double delta_t_;

        // Try-out for quadratic velocity penalization
        Eigen::MatrixXd velocity_control_points_;

        std::vector<Eigen::Vector3d> sampled_points_;

        // bool spline_computed_;
        std::shared_ptr<CubicSpline2D<tk::spline>> path_spline_;
        std::shared_ptr<CubicSpline2D<tk::spline>> trajectory_spline_;

        bool acceleration_weights_computed_;

        /** @brief Compute a 2D path: d -> (x, y), i.e., parameterized in distance */
        void ComputePath();

        Eigen::MatrixXd ComputeVelocitySplinePoints();

        /** @brief Convert an input path to a Trajectory: t -> (x, y) */
        void ConvertToTrajectory(const GeometricPath &path);

        void DebugPrintControlPoints();

        void ComputeAccelerationWeights();
    };

};
#endif // __CUBIC_SPLINE_H__