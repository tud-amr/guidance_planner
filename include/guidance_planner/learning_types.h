#ifndef __LEARNING_TYPES_H__
#define __LEARNING_TYPES_H__

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>

// Learning guidances variables
struct PoseInfo
{
    Eigen::Vector2d position;
    double orientation;
    double velocity;
    ros::Time timestamp;
};
struct PositionTime
{
    Eigen::Vector2d position;
    ros::Time timestamp;
};
struct ObstacleInfo
{
    int id_;
    std::vector<PositionTime> positions_;
    double radius_;
};
#endif // __LEARNING_TYPES_H__