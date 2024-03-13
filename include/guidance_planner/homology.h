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

#include <guidance_planner/topology_comparison.h>
#include <guidance_planner/paths.h>

#include <gsl/gsl_integration.h>

#include <vector>
#include <unordered_map>

namespace GuidancePlanner
{
  class Environment;
  struct Obstacle;

#define GSL_ACCURACY 1e-1 // 1e-1
#define GSL_POINTS 20
  bool operator==(const GeometricPath &a, const GeometricPath &b);

  struct GSLParams;
  class Homology : public TopologyComparison
  {
  public:
    Homology(bool assume_constant_velocity = true);
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

    /** @brief Visualize obstacle and trajectory loops for homology computation */
    void Visualize(Environment &environment) override;

    /** @brief Clear the cache */
    void Clear() override
    {
      cached_values_.clear();
      obstacle_points_ready_ = false;
      obstacle_points_.clear();
    };

  private:
    /** @brief Integrate the H-value over a geometric path (with cached values) */
    double PathHValue(const GeometricPath &path, std::vector<double> &cached_h, const int obstacle_id);

    /** @brief Integrate the H-value in a point over an obstacle */
    double ObstacleHValue(const Eigen::Vector3d &r, const Eigen::Vector3d &dr);

    /** @brief Integrate the H-value in a point over a segment of an obstacle */
    double SegmentHValue(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const Eigen::Vector3d &r, const Eigen::Vector3d &dr);

    void ComputeObstacleLoops(const std::vector<Obstacle> &obstacles);
    void LoadObstacle(int i);

    /** @brief Function that integrates the H value over a segment */
    static double GSLHValue(double x, void *params);

    inline void NumericalIntegration(double &results, void *params);

    static Eigen::Vector3d Line(const Eigen::Vector3d &start, const Eigen::Vector3d &end, double lambda)
    {
      return (1. - lambda) * start + lambda * end;
    };

  private:
    std::vector<gsl_integration_workspace *> gsl_ws_;
    std::vector<gsl_function> gsl_f_;
    std::vector<GSLParams> gsl_params_;

    /** Cached H-Values (over all obstacles) */
    std::unordered_map<GeometricPath, std::vector<double>> cached_values_;

    bool assume_constant_velocity_;
    bool obstacle_points_ready_ = false;
    std::vector<std::vector<Eigen::Vector3d>> obstacle_points_; // For efficiency (4 x num_obstacles)

    /** Parameters for obstacle integration */
    std::vector<Eigen::Vector3d> obstacle_segments_;
    double fraction_;
  };

  struct GSLParams
  {
    Eigen::Vector3d start, end;
    Homology *homology_class;
  };

} // namespace Homotopy
#endif // __HOMOLOGY_H__