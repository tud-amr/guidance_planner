#ifndef __LEARNING_GUIDANCE_H__
#define __LEARNING_GUIDANCE_H__

/**
 * @brief Types for saving history information of obstacles and the robot
 *
 */
#include <guidance_planner/homotopy.h>
#include <guidance_planner/learning_types.h>

#include <guidance_planner/select_guidance.h>

#include <ros/ros.h>
#include <Eigen/Dense>

#include <vector>

namespace RosTools
{
    class Halfspace;
}

namespace GuidancePlanner
{
    class LearningGuidance
    {
    public:
        LearningGuidance();
        void Init(ros::NodeHandle &nh);

    public:
        void SetStart(const Eigen::Vector2d &start, const double orientation, const double velocity);
        void LoadObstacles(const std::vector<Obstacle> &obstacles, const std::vector<RosTools::Halfspace> &static_obstacles);

        bool CallServer(guidance_planner::select_guidance &srv);

        std::vector<PoseInfo> &GetPosesList() { return poses_list_; }
        std::vector<ObstacleInfo> &GetPreviousObstacles() { return previous_obstacles_list_; }

    private:
        ros::ServiceClient select_guidance_client_;
        std::vector<PoseInfo> poses_list_;
        std::vector<ObstacleInfo> previous_obstacles_list_;

        std::vector<Obstacle> obstacles_;
    };
}
#endif // __LEARNING_GUIDANCE_H__