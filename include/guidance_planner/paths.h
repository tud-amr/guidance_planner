#ifndef __PATHS_H__
#define __PATHS_H__

#include <guidance_planner/types.h>

namespace GuidancePlanner
{

  /** @brief Segments help to keep a consistent ID per combination of Guard -> Connector -> Guard, while the Connector ID can vary */
  // Could also just add an extra ID to the node ID, i.e., the path association ID should be a segment association ID
  struct Segment
  {
    int id_;
  };

  /** @brief A collection of nodes that constitute a path */
  struct GeometricPath
  {
    std::vector<Node *> nodes_; // The nodes of this path organized by k

    GeometricPath()
    {
    }
    /** @brief Convert a vector of nodes to a path directly */
    GeometricPath(const std::vector<Node *> &nodes);

    /** @brief Evaluate this path at 0 <= s <= 1
     *  @return Interpolated point in discrete time space (i.e., k in [0, N]) */
    SpaceTimePoint operator()(double s) const;

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

    /** @brief Return this path as a vector of Eigen::Vector3d */
    std::vector<Eigen::Vector3d> AsVectorOfVector3d();

    bool ContainsNode(const Node &node) const;

    friend std::ostream &operator<<(std::ostream &stream, const GeometricPath &path)
    {
      stream << "Path: [";
      for (auto &node : path.nodes_)
        stream << *node << ", ";

      stream << "\b\b]";

      return stream;
    }

    void Clear();

  private:
    std::vector<double> aggregated_distances_; // The distance from start to each node over the path

    void ComputeDistanceVector();
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
      for (auto &node : x.nodes_) // We hash the IDs of the nodes in this path
      {
        int hash_id = node->type_ == GuidancePlanner::NodeType::GOAL ? -10 : node->id_;

        seed ^= (uint32_t)(hash_id) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      }
      return seed;
    }
  };
} // namespace std

#endif // __PATHS_H__