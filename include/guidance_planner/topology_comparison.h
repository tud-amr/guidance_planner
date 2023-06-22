#ifndef __TOPOLOGY_COMPARISON_H__
#define __TOPOLOGY_COMPARISON_H__

#include "guidance_planner/environment.h"
#include "guidance_planner/paths.h"

namespace GuidancePlanner
{
  class TopologyComparison
  {

  public:
    virtual void Visualize(Environment &environment){};
    virtual bool AreEquivalent(const GeometricPath &a, const GeometricPath &b, Environment &environment, bool compute_all = false) = 0;
    virtual std::vector<bool> LeftPassingVector(const GeometricPath &path, Environment &environment) { return std::vector<bool>({}); };

    virtual void Clear(){};
  };
}; // namespace Homotopy

#endif // __TOPOLOGY_COMPARISON_H__