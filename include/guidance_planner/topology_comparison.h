#ifndef __TOPOLOGY_COMPARISON_H__
#define __TOPOLOGY_COMPARISON_H__

#include "guidance_planner/environment.h"
#include "guidance_planner/paths.h"

namespace GuidancePlanner
{
class TopologyComparison
{

public:
  virtual bool AreEquivalent(const GeometricPath &a, const GeometricPath &b, Environment &environment) = 0;
  virtual std::vector<bool> PassesRight(const GeometricPath &path, Environment &environment) { return std::vector<bool>({}); };

  virtual void Clear(){};
};
}; // namespace Homotopy

#endif // __TOPOLOGY_COMPARISON_H__