
#ifndef __WINDING_ANGLE_H__
#define __WINDING_ANGLE_H__

#include <guidance_planner/homotopy_comparison/homotopy_comparison.h>
#include <guidance_planner/types/paths.h>

#include <unordered_map>
#include <vector>

namespace GuidancePlanner
{
    class Environment;
    struct Obstacle;

    class WindingAngle : public HomotopyComparison
    {
    public:
        WindingAngle(bool assume_constant_velocity = true);

    public:
        /** @brief Check if two paths are homotopy equivalent in the given environment */
        virtual bool AreEquivalent(const GeometricPath &a, const GeometricPath &b, Environment &environment, bool compute_all = false) override;

        void Clear() override;

        double pass_threshold_{0.125}; // 1/4 pi
    private:
        bool assume_constant_velocity_;

        std::unordered_map<GeometricPath, std::vector<double>> cached_values_;

        double ComputeWindingAngle(const int obstacle_id,
                                   std::vector<double> &cached_lambda,
                                   const std::vector<Eigen::Vector2d> &robot_prediction,
                                   const std::vector<Eigen::Vector2d> &obstacle_prediction);
    };

} // namespace GuidancePlanner
#endif // __WINDING_ANGLE_H__