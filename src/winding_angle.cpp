#include "guidance_planner/winding_angle.h"
#include "guidance_planner/homology.h"

#include <ros_tools/math.h>
#include <ros_tools/visuals.h>

#include <guidance_planner/environment.h>

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

        // For each obstacle
        for (size_t obstacle_id = 0; obstacle_id < obstacles.size(); obstacle_id++)
        {
            const auto &obstacle = obstacles[obstacle_id];

            double lambda_a = ComputeWindingAngle(obstacle_id, cached_a, a_positions, obstacle.positions_);
            double lambda_b = ComputeWindingAngle(obstacle_id, cached_b, b_positions, obstacle.positions_);

            // Winding angles must be large enough to indicate passing
            if (std::abs(lambda_a) < min_abs_lambda_)
                continue;

            if (std::abs(lambda_b) < min_abs_lambda_)
                continue;

            // If winding angles are different for any obstacle, then the paths are not equivalent
            if (RosTools::sgn(lambda_a) != RosTools::sgn(lambda_b))
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