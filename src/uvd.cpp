#include <guidance_planner/uvd.h>

#include <guidance_planner/paths.h>
#include <guidance_planner/environment.h>

#include <Eigen/Dense>

namespace GuidancePlanner
{
    bool UVD::AreEquivalent(const GeometricPath &a, const GeometricPath &b, Environment &environment, bool compute_all)
    {
        (void)compute_all;
        /** @note Space-time 3D UVD */
        // Instead of a time index, we have a path index to sample over [0-1]. Each sample is a point in 3D space-time
        Eigen::VectorXd path_indices = Eigen::VectorXd::LinSpaced(20, 0., 1.); /** @todo samples as configuration parameter */

        for (int i = 0; i < path_indices.size(); i++) /** @todo exclude start and finish */
        {
            if (!environment.IsVisible(a(path_indices(i)), b(path_indices(i))))
                return false;
        }

        return true; // If no collisions occured - paths are homotopic equivalents
    }
} // namespace GuidancePlanner