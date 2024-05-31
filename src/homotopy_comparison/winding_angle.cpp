#include "guidance_planner/homotopy_comparison/winding_angle.h"

#include <ros_tools/math.h>
#include <ros_tools/visuals.h>

#include <guidance_planner/environment.h>

#include <guidance_planner/types/paths.h>

namespace GuidancePlanner
{
    WindingAngle::WindingAngle(bool assume_constant_velocity)
        : assume_constant_velocity_(assume_constant_velocity)
    {
    }

    /** @brief: https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9954157&casa_token=x7zXpwRfVgEAAAAA:ODmnNaMz5gIHozQKDtdkuvOet9Vs1P4_n0an85_Zy_OF8ODg1_1dP6dc4TBAH1FRpTyuQdsCow&tag=1 */
    /** @note: Does not yet support paths that do not start at 0 time */
    bool WindingAngle::AreEquivalent(const GeometricPath &a, const GeometricPath &b, Environment &environment, bool compute_all)
    {
        (void)compute_all;

        // Retrieve cached h values over the obstacles (i.e., a vector)
        std::vector<double> &cached_a = cached_values_[a]; // Note: will create the cache if it does not exist
        std::vector<double> &cached_b = cached_values_[b]; // Note: will create the cache if it does not exist

        auto &obstacles = environment.GetDynamicObstacles();

        const std::vector<Eigen::Vector2d> a_positions = a.GetKParameterized();
        const std::vector<Eigen::Vector2d> b_positions = b.GetKParameterized();

        int total_a_passes = 0;
        int total_b_passes = 0;

        // For each obstacle
        for (size_t obstacle_id = 0; obstacle_id < obstacles.size(); obstacle_id++)
        {
            const auto &obstacle = obstacles[obstacle_id];

            // Ignore obstacles far away
            // if (RosTools::distance(obstacle.positions_[0], a_positions[0]) > 20. &&
            //     RosTools::distance(obstacle.positions_.back(), a_positions.back()) > 20.)
            // {
            //     continue;
            // }

            // if (RosTools::distance(obstacle.positions_[0], b_positions[0]) > 20. &&
            //     RosTools::distance(obstacle.positions_.back(), b_positions.back()) > 20.)
            // {
            //     continue;
            // }

            double lambda_a = ComputeWindingAngle(obstacle_id, cached_a, a_positions, obstacle.positions_);
            double lambda_b = ComputeWindingAngle(obstacle_id, cached_b, b_positions, obstacle.positions_);

            // Winding angles must be large enough to indicate passing
            bool a_passes = std::abs(lambda_a) >= pass_threshold_;
            bool b_passes = std::abs(lambda_b) >= pass_threshold_;
            if (a_passes)
                total_a_passes++;
            if (b_passes)
                total_b_passes++;

            if (a_passes && b_passes)
            {
                if (RosTools::sgn(lambda_a) != RosTools::sgn(lambda_b))
                {
                    return false;
                }
            }
        }

        if (Config::use_non_passing_) // A path that passes no obstacles is distinct from ones that do
        {
            if (total_a_passes == 0 && total_b_passes != 0)
                return false;

            if (total_b_passes == 0 && total_a_passes != 0)
                return false;
        }

        return true; // If all winding angles are the same for all obstacles, then the paths are equivalent
    }

    double WindingAngle::ComputeWindingAngle(const int obstacle_id,
                                             std::vector<double> &cached_lambda,
                                             const std::vector<Eigen::Vector2d> &robot_prediction,
                                             const std::vector<Eigen::Vector2d> &obstacle_prediction)
    {

        if (obstacle_id < (int)cached_lambda.size())
            return cached_lambda[obstacle_id]; // Retrieve from cache

        double lambda = 0.;
        double prev_angle;

        for (int k = 0; k < Config::N; k++)
        {
            // Compute the winding angle for the obstacle
            Eigen::Vector2d relative_position = robot_prediction[k] - obstacle_prediction[k];
            double angle = std::atan2(relative_position(1), relative_position(0)); // Current angle

            if (k > 1)
                lambda += angle - prev_angle; // Relative angle

            prev_angle = angle;
        }

        cached_lambda.push_back(lambda); // Save to cache

        return lambda;
    }

    void WindingAngle::Clear()
    {
        cached_values_.clear();
    }

} // namespace GuidancePlanner