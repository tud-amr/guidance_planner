#include "guidance_planner/global_guidance.h"
#include "guidance_planner/types.h"
#include <guidance_planner/paths.h>

#include <dynamic_reconfigure/server.h>
#include <guidance_planner/GuidancePlannerConfig.h>
#include <guidance_planner/ObstacleMSG.h>
#include <guidance_planner/TrajectoryMSG.h>
#include <guidance_planner/guidances_Hcost.h>
#include <guidance_planner/guidances.h>

#include <ros/ros.h>

using namespace GuidancePlanner;

void IntegrationExceptionHandler(const char *reason, const char *file, int line, int err)
{
    gsl_stream_printf("ERROR", file, line, reason);
    fflush(stdout);
    fprintf(stderr, "Customized GSL error handler invoked.\n");
    fflush(stderr);

    // abort ();
    throw IntegrationException();
}

class GuidanceServer
{
private:
    ros::NodeHandle nh;
    ros::ServiceServer guidance_srv, guidance_truth_srv, guidance_cost_srv, guidance_Hcost_srv;
    GlobalGuidance guidance;
    GuidancePlanner::Config *config;

public:
    GuidanceServer()
    {
        this->nh = ros::NodeHandle("guidance_server");
        this->guidance_Hcost_srv = this->nh.advertiseService("/get_guidances_Hcost", &GuidanceServer::guidances_Hcost_callback, this);
        this->config = guidance.GetConfig(); // Retrieves the configuration file, if you need it
        gsl_set_error_handler(&IntegrationExceptionHandler);
    }

    bool guidances_callback(guidance_planner::guidances::Request &req, guidance_planner::guidances::Response &res)
    {
        try
        {
            guidance.Reset();
            guidance.SetStart(Eigen::Vector2d(req.x, req.y), req.oriantation, req.v); // Position, yaw angle, velocity magnitude
            std::vector<Goal> goals;
            for (size_t i = 0; i < req.goals_x.size(); i++)
            {
                goals.emplace_back(Eigen::Vector2d(req.goals_x[i], req.goals_y[i]), 1.);
            }
            guidance.SetGoals(goals);
            std::vector<GuidancePlanner::Obstacle> obstacles_vec;
            for (size_t i = 0; i < req.obstacles.size(); i++)
            {
                std::vector<Eigen::Vector2d> obs_pos;
                PRM_LOG("Request obstacle len: " << req.obstacles[i].pos_x.size());
                int last_no_nan = -1;
                for (size_t j = 0; j < req.obstacles[i].pos_x.size(); j++)
                {
                    if (!isnan(req.obstacles[i].pos_x[j]) && !isnan(req.obstacles[i].pos_y[j]))
                    {
                        obs_pos.emplace_back(req.obstacles[i].pos_x[j], req.obstacles[i].pos_y[j]);
                        last_no_nan = j;
                    }
                    else
                    {
                        if (last_no_nan == -1)
                        {
                            j++;
                            while (isnan(req.obstacles[i].pos_x[j]) || isnan(req.obstacles[i].pos_y[j]))
                            {
                                j++;
                            }
                            last_no_nan = j;
                            for (size_t aux_j = 0; aux_j < j; aux_j++)
                            {
                                obs_pos.emplace_back(req.obstacles[i].pos_x[last_no_nan], req.obstacles[i].pos_y[last_no_nan]);
                            }
                        }
                        obs_pos.emplace_back(req.obstacles[i].pos_x[last_no_nan], req.obstacles[i].pos_y[last_no_nan]);
                    }
                }
                obstacles_vec.push_back(GuidancePlanner::Obstacle(req.obstacles[i].id, obs_pos, req.obstacles[i].radius));
            }
            std::vector<Halfspace> static_obstacles;
            for (size_t i = 0; i < req.static_x.size(); i++)
            {
                static_obstacles.emplace_back(Eigen::Vector2d(req.static_x[i], req.static_y[i]), req.static_n[i]);
            }
            guidance.LoadObstacles(obstacles_vec, static_obstacles);
            // Update (i.e., compute) the guidance trajectories
            guidance.Update();
            // Show some results:
            bool success = guidance.Succeeded();
            res.success = success;
            if (success)
            {
                ROS_INFO_STREAM("Guidance planner found: " << guidance.NumberOfGuidanceTrajectories() << " trajectories");
                // for (int i = 0; i < req.n_trajectories; i++){
                for (int i = 0; i < guidance.NumberOfGuidanceTrajectories(); i++)
                {
                    // CubicSpline3D& guidance_spline = guidance.GetGuidanceTrajectory(i%guidance.NumberOfGuidanceTrajectories());
                    CubicSpline3D &guidance_spline = guidance.GetGuidanceTrajectory(i).spline;
                    RosTools::CubicSpline2D<tk::spline> guidance_trajectory = guidance_spline.GetTrajectory(); // Retrieves the trajectory: t -> (x, y))
                    std::vector<double> x_traj, y_traj;
                    for (double t = 0; t <= Config::N * Config::DT; t += Config::DT)
                    {
                        Eigen::Vector2d pos = guidance_trajectory.GetPoint(t);
                        x_traj.emplace_back(pos.x());
                        y_traj.emplace_back(pos.y());
                    }
                    guidance_planner::TrajectoryMSG traj;
                    traj.x = x_traj;
                    traj.y = y_traj;
                    res.trajectories.emplace_back(traj);
                    // ROS_INFO_STREAM("H-cost: " << res.costs.back());
                    guidance_planner::RightAvoidanceMSG h_signature_msg;
                    std::vector<bool> right = guidance.PassesRight(i, Eigen::Vector2d(req.goals_x[0], req.goals_y[0]));
                    for (int i_obs = 0; i_obs < (int)right.size(); i_obs++)
                    {
                        h_signature_msg.right_avoidance.push_back((double)right[i_obs]);
                    }
                    res.h_signature.emplace_back(h_signature_msg);
                    // RosTools::CubicSpline2D<tk::spline> guidance_path = guidance_spline.GetPath(); // Retrieves the path: s -> (x, y)
                }
                // guidance.Visualize();
                // guidance.VisualizePath(truth_path);
                ros::spinOnce();
                // CubicSpline3D& guidance_spline = guidance.GetGuidanceTrajectory(0);

                /** @note If you decide on a used path, you can provide this feedback to the guidance planner and it will remember which path is best */
                // int used_trajectory_id = guidance.GetUsedTrajectory()
                // guidance.SetUsedTrajectory(int spline_id);
            }
            else
            {
                ROS_WARN("\tGuidance planner found no trajectories that reach any of the goals!");
            }
            return true;
        }
        catch (IntegrationException &ie)
        {
            res.success = false;
            ROS_WARN("\tGuidance planner found no trajectories that reach any of the goals!");
            return true;
        }
    }

    bool guidances_Hcost_callback(guidance_planner::guidances_Hcost::Request &req, guidance_planner::guidances_Hcost::Response &res)
    {
        try
        {
            guidance.Reset();
            guidance.SetStart(Eigen::Vector2d(req.x, req.y), req.oriantation, req.v); // Position, yaw angle, velocity magnitude
            std::vector<Goal> goals;
            for (size_t i = 0; i < req.goals_x.size(); i++)
            {
                goals.emplace_back(Eigen::Vector2d(req.goals_x[i], req.goals_y[i]), 1.);
            }
            guidance.SetGoals(goals);
            std::vector<GuidancePlanner::Obstacle> obstacles_vec;
            for (size_t i = 0; i < req.obstacles.size(); i++)
            {
                std::vector<Eigen::Vector2d> obs_pos;
                PRM_LOG("Request obstacle len: " << req.obstacles[i].pos_x.size());
                int last_no_nan = -1;
                for (size_t j = 0; j < req.obstacles[i].pos_x.size(); j++)
                {
                    if (!isnan(req.obstacles[i].pos_x[j]) && !isnan(req.obstacles[i].pos_y[j]))
                    {
                        obs_pos.emplace_back(req.obstacles[i].pos_x[j], req.obstacles[i].pos_y[j]);
                        last_no_nan = j;
                    }
                    else
                    {
                        if (last_no_nan == -1)
                        {
                            j++;
                            while (isnan(req.obstacles[i].pos_x[j]) || isnan(req.obstacles[i].pos_y[j]))
                            {
                                j++;
                            }
                            last_no_nan = j;
                            for (size_t aux_j = 0; aux_j < j; aux_j++)
                            {
                                obs_pos.emplace_back(req.obstacles[i].pos_x[last_no_nan], req.obstacles[i].pos_y[last_no_nan]);
                            }
                        }
                        obs_pos.emplace_back(req.obstacles[i].pos_x[last_no_nan], req.obstacles[i].pos_y[last_no_nan]);
                    }
                }
                obstacles_vec.push_back(GuidancePlanner::Obstacle(req.obstacles[i].id, obs_pos, req.obstacles[i].radius));
            }
            std::vector<Halfspace> static_obstacles;
            for (size_t i = 0; i < req.static_x.size(); i++)
            {
                static_obstacles.emplace_back(Eigen::Vector2d(req.static_x[i], req.static_y[i]), req.static_n[i]);
            }
            guidance.LoadObstacles(obstacles_vec, static_obstacles);
            // Update (i.e., compute) the guidance trajectories
            guidance.Update();
            // Show some results:
            bool success = guidance.Succeeded();
            res.success = success;
            if (success)
            {
                std::vector<GuidancePlanner::Node *> truth_vec_node;
                for (size_t i_node = 0; i_node < req.truth.x.size(); i_node++)
                {
                    SpaceTimePoint point(req.truth.x[i_node], req.truth.y[i_node], i_node);
                    truth_vec_node.emplace_back(new Node(i_node, point, NodeType::NONE));
                }
                GeometricPath truth_path(truth_vec_node);
                ROS_INFO_STREAM("Guidance planner found: " << guidance.NumberOfGuidanceTrajectories() << " trajectories");
                // for (int i = 0; i < req.n_trajectories; i++){
                for (int i = 0; i < guidance.NumberOfGuidanceTrajectories(); i++)
                {
                    // CubicSpline3D& guidance_spline = guidance.GetGuidanceTrajectory(i%guidance.NumberOfGuidanceTrajectories());
                    CubicSpline3D &guidance_spline = guidance.GetGuidanceTrajectory(i).spline;
                    RosTools::CubicSpline2D<tk::spline> guidance_trajectory = guidance_spline.GetTrajectory(); // Retrieves the trajectory: t -> (x, y))
                    std::vector<double> x_traj, y_traj;
                    for (double t = 0; t <= (double)truth_vec_node.size() * Config::DT; t += Config::DT)
                    {
                        Eigen::Vector2d pos = guidance_trajectory.GetPoint(t);
                        x_traj.emplace_back(pos.x());
                        y_traj.emplace_back(pos.y());
                    }
                    guidance_planner::TrajectoryMSG traj;
                    traj.x = x_traj;
                    traj.y = y_traj;
                    res.trajectories.emplace_back(traj);
                    res.costs.emplace_back(guidance.GetHomotopicCost(i, truth_path));
                    ROS_INFO_STREAM("H-cost: " << res.costs.back());
                    guidance_planner::RightAvoidanceMSG h_signature_msg;
                    std::vector<bool> right = guidance.PassesRight(i, Eigen::Vector2d(req.goals_x[0], req.goals_y[0]));
                    for (int i_obs = 0; i_obs < (int)right.size(); i_obs++)
                    {
                        h_signature_msg.right_avoidance.push_back((double)right[i_obs]);
                    }
                    res.h_signature.emplace_back(h_signature_msg);
                    // RosTools::CubicSpline2D<tk::spline> guidance_path = guidance_spline.GetPath(); // Retrieves the path: s -> (x, y)
                }
                // guidance.Visualize();
                // guidance.VisualizePath(truth_path);
                ros::spinOnce();
                // CubicSpline3D& guidance_spline = guidance.GetGuidanceTrajectory(0);

                /** @note If you decide on a used path, you can provide this feedback to the guidance planner and it will remember which path is best */
                // int used_trajectory_id = guidance.GetUsedTrajectory()
                // guidance.SetUsedTrajectory(int spline_id);
            }
            else
            {
                ROS_WARN("\tGuidance planner found no trajectories that reach any of the goals!");
            }
            return true;
        }
        catch (IntegrationException &ie)
        {
            res.success = false;
            ROS_WARN("\tGuidance planner found no trajectories that reach any of the goals!");
            return true;
        }
    }
};

int main(int argc, char **argv)
{
    try
    {
        ROS_INFO("Starting Guidance Server");
        ros::init(argc, argv, ros::this_node::getName());
        GuidanceServer srv;
        ros::spin();
    }
    catch (ros::Exception &e)
    {
        ROS_ERROR("Guidance server: Error occured: %s ", e.what());
        exit(1);
    }

    return 0;
}
