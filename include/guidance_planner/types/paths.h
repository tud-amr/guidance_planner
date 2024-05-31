#ifndef __PATHS_H__
#define __PATHS_H__

#include <guidance_planner/types/types.h>

#include <guidance_planner/types/connection.h>

namespace GuidancePlanner
{
  typedef StraightConnection PathConnection;
  // typedef DubinsConnection PathConnection;

  /** @brief A path that connects several nodes */
  struct GeometricPath
  {
    GeometricPath()
    {
    }
    /** @brief Convert a vector of nodes to a path directly */
    GeometricPath(const std::vector<Node *> &nodes);

    GeometricPath(const GeometricPath &other)
    {
      connections_ = other.GetConnections();
      ComputeDistanceVector();
    }
    GeometricPath &operator=(const GeometricPath &other)
    {
      connections_ = other.GetConnections();
      ComputeDistanceVector();
      return *this;
    }

    /** @brief Evaluate this path at 0 <= s <= 1
     *  @return Interpolated point in discrete time space (i.e., k in [0, N]) */
    SpaceTimePoint operator()(double s) const;

    bool isValid(Config *config, const Eigen::Vector2d &start_velocity, double start_orientation);

    std::vector<Eigen::Vector2d> GetKParameterized() const;

    double Length2D() const;
    double Length3D() const;

    /** @brief Return a smoothness measure, which divides the distance between start and end by the 3D length of the path. I.e., straighter paths are
     * better */
    double RelativeSmoothness() const;
    double AverageVelocity() const;

    /** @brief Loop through nodes, get the lowest "k" */
    double StartTimeIndex() const;

    /** @brief Loop through nodes, get the highest "k" */
    double EndTimeIndex() const;

    Node *GetStart() const;
    Node *GetEnd() const;

    const std::vector<PathConnection> &GetConnections() const { return connections_; }
    std::vector<Node *> GetNodes() const;
    std::vector<SpaceTimePoint> GetIntegrationNodes() const;

    /** @brief Return this path as a vector of Eigen::Vector3d */
    std::vector<Eigen::Vector3d> AsVectorOfVector3d();

    bool ContainsNode(const Node &node) const;

    friend std::ostream &operator<<(std::ostream &stream, const GeometricPath &path);

    void Clear();

  private:
    std::vector<double> aggregated_distances_; // The distance from start to each node over the path

    void ComputeDistanceVector();

    std::vector<PathConnection> connections_;
    bool validity_checked_{false};
    bool valid_{false};
  };

  bool operator==(const GeometricPath &a, const GeometricPath &b);

  struct StandaloneGeometricPath
  {
    GeometricPath path;
    std::list<Node> saved_nodes_;

    StandaloneGeometricPath()
    {
    }

    StandaloneGeometricPath(const std::list<Node> &nodes);
    StandaloneGeometricPath(const GeometricPath &other);

    // To cast to a GeometricPath
    operator GeometricPath &();
  };

  class IDAssigner
  {
  public:
    IDAssigner(int num_objects);

  public:
    int GetID();
    void MarkIDAsUsed(int ID);

  private:
    std::vector<bool> IDs_;
  };
}

/** Overwrite hash functionality for paths! **/
namespace std
{
  template <>
  struct hash<GuidancePlanner::GeometricPath>
  {
    size_t operator()(const GuidancePlanner::GeometricPath &x) const
    {
      size_t seed = 0;
      for (auto &node : x.GetNodes()) // We hash the IDs of the nodes in this path
      {
        int hash_id = node->type_ == GuidancePlanner::NodeType::GOAL ? -10 : node->id_;

        seed ^= (uint32_t)(hash_id) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      }

      return seed;
    }
  };
} // namespace std

#endif // __PATHS_H__