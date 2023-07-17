#include <guidance_planner/learning_guidance.h>

#include <ros_tools/types.h>

#include <guidance_planner/ObstacleMSG.h>
#include <guidance_planner/TrajectoryMSG.h>
#include <guidance_planner/RightAvoidanceMSG.h>

namespace GuidancePlanner
{
    LearningGuidance::LearningGuidance(){};

    void LearningGuidance::Init(ros::NodeHandle &nh)
    {
        /* Initialize service client */
        select_guidance_client_ = nh.serviceClient<guidance_planner::select_guidance>("/select_guidance");
    }

    bool LearningGuidance::CallServer(guidance_planner::select_guidance &srv)
    {
        return select_guidance_client_.call(srv);
    }

    void LearningGuidance::SetStart(const Eigen::Vector2d &start, const double orientation, const double velocity)
    {
        ros::Time timestamp = ros::Time::now();

        if (poses_list_.size() == 0 || (timestamp > poses_list_.back().timestamp && (timestamp - poses_list_.back().timestamp).toSec() > 0.09))
        {
            // Trained with a history of 2.8 s
            while (poses_list_.size() > 1 && (timestamp - poses_list_[1].timestamp >= ros::Duration(2.8)))
            {
                // Erase the first (oldest) position
                poses_list_.erase(poses_list_.begin());
            }
            PoseInfo pose_new;
            pose_new.orientation = orientation;
            pose_new.position = start;
            pose_new.timestamp = timestamp;
            pose_new.velocity = velocity;
            poses_list_.push_back(pose_new);
        }
    }

    void LearningGuidance::LoadObstacles(const std::vector<Obstacle> &obstacles, const std::vector<RosTools::Halfspace> &static_obstacles)
    {
        obstacles_ = obstacles; // Local copy

        ros::Time timestamp = ros::Time::now();
        // Iterate over the current obstacles
        for (const auto &currentObstacle : obstacles)
        {
            // Check if the obstacle exists in the original vector
            auto it = std::find_if(previous_obstacles_list_.begin(), previous_obstacles_list_.end(),
                                   [&](const ObstacleInfo &o)
                                   {
                                       return o.id_ == currentObstacle.id_;
                                   });
            // If the obstacle doesn't exist in the original vector, add it
            if (it == previous_obstacles_list_.end())
            {
                ObstacleInfo obs_new;
                obs_new.id_ = currentObstacle.id_;
                obs_new.radius_ = currentObstacle.radius_;
                PositionTime pos_new;
                pos_new.timestamp = timestamp;
                pos_new.position = currentObstacle.positions_[0];
                obs_new.positions_.push_back(pos_new);
                previous_obstacles_list_.push_back(obs_new);
            }
            // Update the positions of the existing obstacle
            else
            {
                // Check time interval and how many to store
                // ROS_INFO_STREAM("Obstacle id: " << it->id_ << ". Timestamp: " << timestamp);
                if (timestamp > it->positions_.back().timestamp && (timestamp - it->positions_.back().timestamp).toSec() > 0.09)
                {
                    while (it->positions_.size() > 1 && (timestamp - it->positions_[1].timestamp >= ros::Duration(2.8)))
                    {
                        // Erase the first (oldest) position
                        it->positions_.erase(it->positions_.begin());
                    }
                    PositionTime pos_new;
                    pos_new.timestamp = timestamp;
                    pos_new.position = currentObstacle.positions_[0];
                    it->positions_.push_back(pos_new);
                }
                // if (it->id_ == 0){
                //   ROS_INFO_STREAM("-----------------Obstacle 0--------------------");
                //   for (size_t i_time = 0; i_time<it->positions_.size(); i_time++){
                //     ROS_INFO_STREAM("Obstacle i: " << i_time << ". Timestamp: " << it->positions_[i_time].timestamp);
                //   }
                // }
            }
        }

        // Iterate over the original obstacles and remove the ones that don't appear in the current vector
        previous_obstacles_list_.erase(std::remove_if(previous_obstacles_list_.begin(), previous_obstacles_list_.end(),
                                                      [&](const ObstacleInfo &o)
                                                      {
                                                          auto it = std::find_if(obstacles.begin(), obstacles.end(),
                                                                                 [&](const Obstacle &co)
                                                                                 {
                                                                                     return co.id_ == o.id_;
                                                                                 });
                                                          return it == obstacles.end();
                                                      }),
                                       previous_obstacles_list_.end());
        // Sort by id so that previous positions and homology classes have the same order
        std::sort(obstacles_.begin(), obstacles_.end(),
                  [&](const GuidancePlanner::Obstacle &a, const GuidancePlanner::Obstacle &b) -> bool
                  {
                      return a.id_ > b.id_;
                  });
        std::sort(previous_obstacles_list_.begin(), previous_obstacles_list_.end(),
                  [&](const ObstacleInfo &a, const ObstacleInfo &b) -> bool
                  {
                      return a.id_ > b.id_;
                  });
    }

};