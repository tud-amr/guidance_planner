#include <ros/ros.h>
#include <string>
#include <stdexcept>
#include "guidance_planner/global_guidance.h"
#include "guidance_planner/types.h"

// Reading from file
#include <20_ts/read_scenario.h>

using namespace GuidancePlanner;

Config *config_;
bool replan_ = false;

// bool random_obstacles_ = false; // Set to true to randomize the obstacles
// std::string obstacle_source = "Random";
// std::string obstacle_source = "Manual";
std::string obstacle_source = "File";

int count = 0;
int file_id = 0;

/** @brief Define obstacles manually */
void ManualObstacles(std::vector<GuidancePlanner::Obstacle> &obstacles)
{
    obstacles.clear();

    /** @brief For constant velocity obstacles */
    /* Obstacle ID, position, velocity in 2D, integrator timestep, number of steps, radius */
    // obstacles.emplace_back(0, Eigen::Vector2d(3.5, -2), Eigen::Vector2d(0, 1.0), Config::DT, Config::N, 0.5);
    obstacles.emplace_back(0, Eigen::Vector2d(2.5, 2), Eigen::Vector2d(0, -1.0), Config::DT, Config::N, 0.5);

    /** @note If you have different predictions, use the following */
    /* Obstacle ID, std::vector<Eigen::Vector2d> &positions -> first position must be the CURRENT position!, radius */
    std::vector<Eigen::Vector2d> positions;
    for (int k = 0; k < Config::N + 1; k++)
    {
        if (k == 0)
            positions.emplace_back(3.5, -2);
        else
            positions.emplace_back(positions[k - 1] + Eigen::Vector2d(0., 1.) * Config::DT);
    }
    obstacles.emplace_back(1, positions, 0.5);
}

void RandomizeObstacles(std::vector<GuidancePlanner::Obstacle> &obstacles)
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

void FileObstacles(GlobalGuidance &guidance, std::vector<GuidancePlanner::Obstacle> &obstacles)
{
    int num_files = 80;
    obstacles.clear();

    int i = 0;
    if (count % 10 == 0)
    {
        file_id++;
        if (file_id >= num_files)
            file_id = file_id % num_files;
    }
    ReadFile("scenario_" + std::to_string(file_id) + ".bin", guidance, obstacles);

    count++;
}

int main(int argc, char **argv)
{
    try
    {
        ROS_INFO("Starting Homotopy Test");
        ros::init(argc, argv, ros::this_node::getName());

        ROS_INFO("Creating Guidance");
        GlobalGuidance guidance;        // Initializes the guidance planner
        config_ = guidance.GetConfig(); // Retrieves the configuration file, if you need it

        /** @brief Load inputs for the guidance planner (you should do this for each computation) */
        ROS_INFO("Preparing Obstacles");
        std::vector<Obstacle> obstacles;

        /** @brief Static obstacles: Ax <= b */
        std::vector<RosTools::Halfspace> static_obstacles;
        static_obstacles.resize(2);
        static_obstacles.emplace_back(Eigen::Vector2d(0., 1.), 10.);  // y <= 10
        static_obstacles.emplace_back(Eigen::Vector2d(0., -1.), 10.); // y >= -10

        /** @brief Set the robot position */
        guidance.SetStart(Eigen::Vector2d(0., 0.), 0., 2.); // Position, yaw angle, velocity magnitude

        /** @brief Set the goals for the planner */
        std::vector<Goal> goals;

        // Using explicit goals
        // goals.emplace_back(Eigen::Vector2d(6., 0.), 1.); // x = 6, y = 0, cost = 1
        // goals.emplace_back(Eigen::Vector2d(8., 0.), 1.); // x = 8, y = 0, cost = 1
        // guidance.SetGoals(goals);

        // Using a reference path
        // Construct a spline in x, y
        if (obstacle_source != "File")
        {
            std::vector<double> xx, yy, ss;
            xx = {0., 2., 4., 6., 8., 10.};
            yy = {0., 0., 0., 0., 0., 0.};
            ss = {0., 2., 4., 6., 8., 10.};
            tk::spline x_spline, y_spline;
            x_spline.set_points(ss, xx);
            y_spline.set_points(ss, yy);
            std::unique_ptr<RosTools::CubicSpline2D<tk::spline>> reference_path;
            reference_path.reset(new RosTools::CubicSpline2D<tk::spline>(x_spline, y_spline));

            double distance_on_spline = 0.;
            guidance.LoadReferencePath(distance_on_spline, reference_path);
        }
        // ----------------------------
        /** @brief Mimic a control loop */
        ros::Rate rate(5);
        while (!ros::isShuttingDown())
        {
            ROS_WARN("Updating Guidance");
            if (obstacle_source == "Random")
                RandomizeObstacles(obstacles);
            else if (obstacle_source == "File")
            {
                FileObstacles(guidance, obstacles);
                ros::spinOnce();
                rate.sleep();

                continue;
            }
            else
                ManualObstacles(obstacles);

            // Normally: load the start and goals in each timestep
            // Then, load the obstacles
            guidance.LoadObstacles(obstacles, static_obstacles);

            // Update (i.e., compute) the guidance trajectories
            guidance.Update();

            // Show some results:
            bool success = guidance.Succeeded();
            if (success)
            {
                ROS_INFO_STREAM("Guidance planner found: " << guidance.NumberOfGuidanceTrajectories() << " trajectories");
                CubicSpline3D &guidance_spline = guidance.GetGuidanceTrajectory(0);
                ROS_INFO("[Best Trajectory]");

                RosTools::CubicSpline2D<tk::spline> guidance_trajectory = guidance_spline.GetTrajectory(); // Retrieves the trajectory: t -> (x, y))
                for (double t = 0; t < Config::N * Config::DT; t += 4 * Config::DT)
                {
                    Eigen::Vector2d pos = guidance_trajectory.GetPoint(t);
                    ROS_INFO_STREAM("\t[t = " << t << "]: (" << pos(0) << ", " << pos(1) << ")");
                }
                RosTools::CubicSpline2D<tk::spline> guidance_path = guidance_spline.GetPath(); // Retrieves the path: s -> (x, y)

                /** @note If you decide on a used path, you can provide this feedback to the guidance planner and it will remember which path is best */
                // int used_trajectory_id = guidance.GetUsedTrajectory()
                // guidance.SetUsedTrajectory(int spline_id);
            }
            else
            {
                ROS_WARN("\tGuidance planner found no trajectories that reach any of the goals!");
            }

            replan_ = false;

            while (!ros::isShuttingDown() && !replan_)
            {
                guidance.Visualize(); // Visualize the result in RVIZ
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
