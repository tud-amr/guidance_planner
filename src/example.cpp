#include <ros/ros.h>
#include "guidance_planner/global_guidance.h"
#include "guidance_planner/types.h"

#include <dynamic_reconfigure/server.h>
#include <guidance_planner/GuidancePlannerConfig.h>


using namespace GuidancePlanner;

boost::shared_ptr<dynamic_reconfigure::Server<GuidancePlannerConfig>> reconfigure_server_;
boost::recursive_mutex reconfig_mutex_;

Config *config_;

bool first_reconfigure_callback_ = true;
bool replan_ = false;

bool random_obstacles_ = false; // Set to true to randomize the obstacles

// Mainly for debugging purposes (not in the namespace, to use lmpcc stuff)
void GuidancePlannerTestReconfigureCallback(GuidancePlannerConfig &config, uint32_t level)
{
    if (first_reconfigure_callback_) // Set the reconfiguration parameters to match the yaml configuration at startup
    {
        first_reconfigure_callback_ = false;

        config.debug = Config::debug_output_;

        config.n_paths = config_->n_paths_;
        config.n_samples = config_->n_samples_;

        config.geometric = config_->geometric_weight_;
        config.smoothness = config_->smoothness_weight_;
        config.collision = config_->collision_weight_;
        config.repeat_times = config_->repeat_times_;

        config.spline_consistency = config_->selection_weight_consistency_;
    }

    Config::debug_output_ = config.debug;

    config_->n_paths_ = config.n_paths;
    config_->n_samples_ = config.n_samples;

    config_->geometric_weight_ = config.geometric;
    config_->smoothness_weight_ = config.smoothness;
    config_->collision_weight_ = config.collision;
    config_->repeat_times_ = config.repeat_times;

    config_->selection_weight_consistency_ = config.spline_consistency;

    if (config.replan)
    {
        config.replan = false;
        replan_ = true;
    }
}

void ManualObstacles(std::vector<GuidancePlanner::Obstacle> &obstacles)
{
    obstacles.clear();
    obstacles.emplace_back(0, Eigen::Vector2d(3.5, -2), Eigen::Vector2d(0, 1.0), Config::DT, Config::N, 0.5);
    obstacles.emplace_back(1, Eigen::Vector2d(2.5, 2), Eigen::Vector2d(0, -1.0), Config::DT, Config::N, 0.5);
}

void RandomizeObstacles(std::vector<GuidancePlanner::Obstacle> &obstacles) // Only for standalone PRM
{
    obstacles.clear();
    RosTools::RandomGenerator obstacle_randomizer;
    for (int i = 0; i < 6; i++)
    {
        Eigen::Vector2d start(2.0 + obstacle_randomizer.Double() * 4.0, -2. + 2. * obstacle_randomizer.Double());
        Eigen::Vector2d goal(2.0 + obstacle_randomizer.Double() * 4.0, -2. + 2. * obstacle_randomizer.Double());
        Eigen::Vector2d vel = (goal - start).normalized();

        obstacles.emplace_back(i, start, vel, Config::DT, Config::N, 0.5);
    }
}

int main(int argc, char **argv)
{
    try
    {
        ROS_INFO("Starting Homotopy Test");

        ros::init(argc, argv, ros::this_node::getName());

        // Test PRM!
        ROS_INFO("Creating Guidance");
        GlobalGuidance guidance;
        config_ = guidance.GetConfig();

        ros::NodeHandle nh_predictive("predictive_controller");
        reconfigure_server_.reset(new dynamic_reconfigure::Server<GuidancePlanner::GuidancePlannerConfig>(reconfig_mutex_, nh_predictive));
        reconfigure_server_->setCallback(boost::bind(&GuidancePlannerTestReconfigureCallback, _1, _2));

        ROS_INFO("Preparing Obstacles");
        std::vector<Obstacle> obstacles;
        std::vector<std::vector<Halfspace>> static_obstacles;
        static_obstacles.resize(Config::N);

        guidance.SetStart(Eigen::Vector2d(0., 0.), 0., 2.);
        std::vector<Goal> goals;
        goals.emplace_back(Eigen::Vector2d(6., 0.), 1.);
        goals.emplace_back(Eigen::Vector2d(8., 0.), 1.);
        guidance.SetGoals(goals);

        ros::Rate rate(5);
        while (!ros::isShuttingDown())
        {
            ROS_WARN("Updating Guidance");
            if (random_obstacles_)
                RandomizeObstacles(obstacles);
            else
                ManualObstacles(obstacles);

            guidance.LoadObstacles(obstacles, static_obstacles);

            guidance.Update();
            replan_ = false;

            while (!ros::isShuttingDown() && !replan_)
            {
                guidance.Visualize();
                if (config_->debug_continuous_replanning_)
                    replan_ = true;

                ros::spinOnce();
                rate.sleep();
            }
        }
    }
    catch (ros::Exception &e)
    {
        ROS_ERROR("Test Homotopy Node: Error occured: %s ", e.what());
        exit(1);
    }

    return 0;
}
