/**
 * @file environment.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Define collision checker and homotopy verification functionality in 2D x [0, T]
 * @version 0.1
 * @date 2023-01-02
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__

#include <guidance_planner/types/types.h>

#include <Eigen/Dense>

#include <vector>

namespace GuidancePlanner
{
  // struct Node;
  // struct SpaceTimePoint;
  struct Obstacle;
  struct Halfspace;

  /**
   * @brief This class should make it easier to define different collision checks or state spaces
   *
   */
  class Environment
  {
  public:
    Environment(){};

    virtual void Init();

  public:
    virtual void LoadObstacles(const std::vector<Obstacle> &dynamic_obstacles, const std::vector<Halfspace> &static_obstacles);
    virtual void SetPosition(const Eigen::Vector2d &pos) { (void)pos; };

    /** @brief Check if a point is in collision */
    virtual bool InCollision(const SpaceTimePoint &point, double with_margin = 0.);

    /** @brief Check if the line from point_one to point_two is collision free */
    virtual bool IsVisible(const SpaceTimePoint &point_one, const SpaceTimePoint &point_two);
    virtual bool IsVisible(const Node &a, const Node &b); // Wrapper

    /** @brief Project a point from the obstacles with an additional margin */
    virtual void ProjectToFreeSpace(Eigen::Vector2d &point, int k, double with_margin = 0.);
    virtual void ProjectToFreeSpace(SpaceTimePoint &point, double with_margin = 0.); // wrapper

    /** @brief Only for dynamic obstacles and deprecated. 2-D version of IsVisibility for a particular time */
    virtual bool IsVisible2D(const SpaceTimePoint &point_one, const SpaceTimePoint &point_two);

    virtual std::vector<Obstacle> &GetDynamicObstacles() { return dynamic_obstacles_; };

  protected:
    std::vector<Obstacle> dynamic_obstacles_;
    std::vector<Halfspace> static_obstacles_;

    /** @brief Various implementations of visibility checks */
    virtual bool IsVisibleRayCast(const SpaceTimePoint &point_one, const SpaceTimePoint &point_two); // Fast for constant velocity prediction
    virtual bool IsVisibleRaySampling(const SpaceTimePoint &point_one, const SpaceTimePoint &point_two);
  };

  class GriddedEnvironment : public Environment
  {
  public:
    virtual void Init() override;
    virtual void SetPosition(const Eigen::Vector2d &pos) override { grid_.SetOrigin(pos); };
    virtual void LoadObstacles(const std::vector<Obstacle> &dynamic_obstacles,
                               const std::vector<Halfspace> &static_obstacles) override;

    virtual bool InCollision(const SpaceTimePoint &point, double with_margin = 0.) override;

  private:
    /** @brief Struct to store useful data when gridding obstacles */
    struct SingleObstacle
    {
      Eigen::Vector2d position;
      double radius;

      SingleObstacle(const Eigen::Vector2d &pos, const double r)
      {
        position = pos;
        radius = r;
      }
    };

    class ObstacleGrid
    {

    public:
      ObstacleGrid(){};

      /** @brief Set the dimensions of the grid (in m) and resize the grid data */
      void SetDimensions(double length, double width, double obstacle_radius = 0.5);
      /** @brief Set the origin of the grid
       * @note We can ignore orientation as long as the grid is roughly square */
      void SetOrigin(const Eigen::Vector2d &pos);

      /** @brief Clear all obstacle lists in the grid */
      void Clear();

      /** @brief Insert an obstacle at k into the grid */
      void InsertObstacle(int k, const SingleObstacle &obstacle);

      /** @brief Retrieve all obstacles at the given point */
      std::vector<SingleObstacle> &ObstaclesAt(const SpaceTimePoint &point);

      void Print();

    private:
      std::vector<std::vector<std::vector<std::vector<SingleObstacle>>>> grid_; // t X (x, y) X obstacles
      Eigen::Vector2d origin_;
      double grid_size_;

      std::vector<SingleObstacle> empty_obstacle_list_;

      int size_x_ = 0, size_y_ = 0;

      bool OutOfGridBounds(int i_x, int i_y);                                      // Are the given indices outside of the grid?
      void GetGridIndices(const Eigen::Vector2d &pos, int &i_x_out, int &i_y_out); // Convert (x, y) to grid indices i_x, i_y
    };

  private:
    ObstacleGrid grid_;
  };

} // namespace Homotopy
#endif // __ENVIRONMENT_H__