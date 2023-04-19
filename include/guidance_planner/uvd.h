#ifndef __UVD_H__
#define __UVD_H__

#include <Eigen/Dense>

#include "guidance_planner/topology_comparison.h"

namespace GuidancePlanner
{
class UVD : public TopologyComparison
{
  virtual bool AreEquivalent(const GeometricPath &a, const GeometricPath &b, Environment &environment) override
  {
    /** @note Space-time 3D UVD */
    // Instead of a time index, we have a path index to sample over [0-1]. Each sample is a point in 3D space-time
    Eigen::VectorXd path_indices = Eigen::VectorXd::LinSpaced(20, 0., 1.); /** @todo samples as configuration parameter */

    for (int i = 0; i < path_indices.size(); i++) /** @todo exclude start and finish */
    {
      if (!environment.IsVisible(a(path_indices(i)), b(path_indices(i))))
        return false;
    }

    return true; // If no collisions occured - paths are homotopic equivalents
  };
};
} // namespace Homotopy

// DISABLED UVD HOMOTOPY CHECK
// Homotopy check with visualization
// if (config_->debug_output_)
// {
//   ROSLine &line = debug_visuals_->getNewLine();
//   line.setScale(0.05, 0.05);
//   /** @note Space-time 3D UVD */
//   // Instead of a time index, we have a path index to sample over [0-1]. Each sample is a point in 3D space-time
//   Eigen::VectorXd path_indices = Eigen::VectorXd::LinSpaced(20, 0., 1.); /** @todo samples as configuration parameter */

//   for (int i = 0; i < path_indices.size(); i++) /** @todo exclude start and finish */
//   {
//     if (config_->debug_output_ && done_) // Plot in the PRM (!done) or the filter stage (done)
//     {
//       if (environment_.IsVisible(a(path_indices(i)), b(path_indices(i))))
//         line.setColor(0., 1., 0.); // Green = success
//       else
//         line.setColor(1., 0., 0.); // Red = failure

//       // Get the current points on both paths in continuous time
//       Eigen::Vector3d a_val = a(path_indices(i)).MapToTime();
//       Eigen::Vector3d b_val = b(path_indices(i)).MapToTime(); // Time index must have "k"
//       line.addLine(a_val, b_val);
//     }

//     if (!environment_.IsVisible(a(path_indices(i)), b(path_indices(i))))
//       return false;
//   }
// }

#endif // __UVD_H__
