/**
 * @file homology.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Implements 3-D H-value integration over trajectories for comparing the homology of two trajectories.
 * @version 0.1
 * @date 2023-03-27
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef __HOMOLOGY_H__
#define __HOMOLOGY_H__

#include <functional>

#include "gsl/gsl_integration.h"

#include "guidance_planner/environment.h"
#include "guidance_planner/paths.h"
#include "guidance_planner/topology_comparison.h"

namespace GuidancePlanner
{
#define GSL_ACCURACY 1e-1 // 1e-1
#define GSL_POINTS 20
  bool operator==(const GeometricPath &a, const GeometricPath &b);

  struct GSLParams;
  class Homology : public TopologyComparison
  {
  public:
    Homology();
    virtual ~Homology();

  public:
    /** @brief Check if two paths are homotopy equivalent in the given environment */
    virtual bool AreEquivalent(const GeometricPath &a, const GeometricPath &b, Environment &environment, bool compute_all = false) override;
    /** @brief Get the cost between two paths in the given environment */
    double GetCost(const GeometricPath &a, const GeometricPath &b, Environment &environment);

    /**
     * @brief Check if the path passes left or right by comparing with a "left-always" trajectory. Relies on a somewhat heuristic definition of what is "left".
     *
     * @param path Path through the state space
     * @param environment Obstacle environment
     * @return std::vector<bool> For each obstacle, if it is passed left (=true) or right (=false)
     */
    virtual std::vector<bool> LeftPassingVector(const GeometricPath &path, Environment &environment) override;

    /** @brief Clear the cache */
    void Clear() override { cached_values_.clear(); };

  private:
    std::vector<gsl_integration_workspace *> gsl_ws_;
    std::vector<gsl_function> gsl_f_;
    std::vector<GSLParams> gsl_params_;

    /** Cached H-Values (over all obstacles) */
    std::unordered_map<GeometricPath, std::vector<double>> cached_values_;

    /** @brief Integrate the H-value over a geometric path (with cached values) */
    double PathHValue(const GeometricPath &path, std::vector<double> &cached_h, const int obstacle_id, const Obstacle &obstacle);

    /** @brief Integrate the H-value in a point over an obstacle */
    double ObstacleHValue(const Eigen::Vector3d &r, const Eigen::Vector3d &dr);

    /** @brief Integrate the H-value in a point over a segment of an obstacle */
    double SegmentHValue(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const Eigen::Vector3d &r, const Eigen::Vector3d &dr);

    void ComputeObstacleLoop(const Obstacle &obstacle);

    /** @brief Function that integrates the H value over a segment */
    static double GSLHValue(double x, void *params);

    inline void NumericalIntegration(double &results, void *params);

    static Eigen::Vector3d Line(const Eigen::Vector3d &start, const Eigen::Vector3d &end, double lambda)
    {
      return (1. - lambda) * start + lambda * end;
    };

    /** Parameters for obstacle integration */
    Eigen::Vector3d start_, end_;
    Eigen::Vector3d obstacle_p1_, obstacle_p2_, obstacle_p3_, obstacle_p4_;
    double fraction_;
  };

  struct GSLParams
  {
    Eigen::Vector3d start, end;
    Homology *homology_class;
  };

};     // namespace Homotopy
#endif // __HOMOLOGY_H__