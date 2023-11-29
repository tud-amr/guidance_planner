#ifndef __TOPOLOGY_COMPARISON_H__
#define __TOPOLOGY_COMPARISON_H__

#include <vector>

namespace GuidancePlanner
{
  class Environment;
  class GeometricPath;

  class TopologyComparison
  {

  public:
    virtual bool AreEquivalent(const GeometricPath &a, const GeometricPath &b, Environment &environment, bool compute_all = false) = 0;
    virtual std::vector<bool> LeftPassingVector(const GeometricPath &path, Environment &environment) { return std::vector<bool>({}); };

    virtual void Visualize(Environment &environment){};
    virtual void Clear(){};
  };

  /** @brief To disable topology comparison, you can use this topology comparison instead */
  class NoTopologyComparison : public TopologyComparison
  {
    // There are no equivalences
    bool AreEquivalent(const GeometricPath &a, const GeometricPath &b, Environment &environment, bool compute_all = false) override { return false; }
  };
}; // namespace Homotopy

#endif // __TOPOLOGY_COMPARISON_H__