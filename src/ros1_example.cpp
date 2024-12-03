#include <ros/ros.h>
#include <guidance_planner/global_guidance.h>

#include <ros_tools/profiling.h>
#include <ros_tools/logging.h>
#include <ros_tools/visuals.h>
#include <ros_tools/convertions.h>

#include <string>
#include <stdexcept>

using namespace GuidancePlanner;

Config *config_;
bool replan_ = false;

std::string obstacle_source = "Random";
// std::string obstacle_source = "Manual";

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
    auto &publisher = VISUALS.getPublisher("people");
    auto &model = publisher.getNewModelMarker();
    model.setColor(25. / 256., 138. / 256., 89. / 256.);

    auto &line = publisher.getNewLine();
    line.setColor(25. / 256., 138. / 256., 89. / 256., 1.0);
    line.setScale(0.1, 0.1);

    int num_obstacles = 4;
    obstacles.clear();
    RosTools::RandomGenerator obstacle_randomizer;

    for (int i = 0; i < num_obstacles; i++)
    {
        Eigen::Vector2d start(3.0 + obstacle_randomizer.Double() * 4.0, -3. + 6. * obstacle_randomizer.Double());
        Eigen::Vector2d goal(-2.0 + obstacle_randomizer.Double() * 4.0, -3. + 6. * obstacle_randomizer.Double());
        Eigen::Vector2d vel = (goal - start).normalized();

        obstacles.emplace_back(i, start, vel, Config::DT, Config::N, 0.5);

        model.setOrientation(RosTools::angleToQuaternion(std::atan2(vel(1), vel(0)) + M_PI_2));
        model.addPointMarker(start);

        Eigen::Vector3d cur(start(0), start(1), 0.);
        Eigen::Vector3d prev = cur;
        for (int k = 0; k < Config::N; k++)
        {
            cur += Eigen::Vector3d(vel(0) * Config::DT, vel(1) * Config::DT, Config::DT);
            line.addLine(prev, cur);
            prev = cur;
        }
    }
    publisher.publish();
}

int main(int argc, char **argv)
{

    ROS_INFO("Starting Homotopy Test");
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh;
    VISUALS.init(&nh);

    // Initialize profiling
    RosTools::Instrumentor::Get().BeginSession("guidance_planner");

    LOG_INFO("Creating Guidance");
    GlobalGuidance guidance;        // Initializes the guidance planner
    config_ = guidance.GetConfig(); // Retrieves the configuration file, if you need it

    /** @brief Load inputs for the guidance planner (you should do this for each computation) */
    ROS_INFO("Preparing Obstacles");
    std::vector<Obstacle> obstacles;

    /** @brief Static obstacles: Ax <= b */
    std::vector<Halfspace> static_obstacles;
    static_obstacles.emplace_back(Eigen::Vector2d(0., 1.), 10.);  // y <= 10
    static_obstacles.emplace_back(Eigen::Vector2d(0., -1.), 10.); // y >= -10

    /** @brief Set the robot position */
    guidance.SetStart(Eigen::Vector2d(0., 0.), 0., 2.); // Position, yaw angle, velocity magnitude

    /** @brief Set the goals for the planner */
    bool use_reference_path = true;

    if (use_reference_path)
    {
        // Using a reference path in 2D as a RosTools::Spline2D object
        // Construct a spline in x, y
        std::vector<double> xx, yy;
        xx = {0., 2., 4., 6., 8., 10.};
        yy = {0., 0., 0., 0., 0., 0.};
        std::shared_ptr<RosTools::Spline2D> reference_path = std::make_shared<RosTools::Spline2D>(xx, yy);

        double distance_on_spline = 0.; // Where we are on the spline
        double road_width = 6.;
        guidance.LoadReferencePath(distance_on_spline, reference_path, road_width);
    }
    else
    {
        // Using explicit goals as a set of 2D points with costs
        std::vector<Goal> goals;
        goals.emplace_back(Eigen::Vector2d(6., 0.), 1.); // x = 6, y = 0, cost = 1
        goals.emplace_back(Eigen::Vector2d(8., 0.), 1.); // x = 8, y = 0, cost = 1
        guidance.SetGoals(goals);
    }

    // ----------------------------
    auto &benchmarker = BENCHMARKERS.getBenchmarker("Guidance Planning");

    /** @brief Mimic a control loop */
    ros::Rate rate(1. / 1.); // 1s
    while (!ros::isShuttingDown())
    {
        ROS_WARN("Updating Guidance");
        if (obstacle_source == "Random")
            RandomizeObstacles(obstacles);
        else
            ManualObstacles(obstacles);

        benchmarker.start();

        // Normally: load here the start and goals in each timestep

        // Then, load the obstacles
        guidance.LoadObstacles(obstacles, static_obstacles);

        // Compute the guidance trajectories
        guidance.Update();

        benchmarker.stop();

        // Show some results:
        bool success = guidance.Succeeded();
        if (success)
        {
            ROS_INFO_STREAM("Guidance planner found: " << guidance.NumberOfGuidanceTrajectories() << " trajectories");
            CubicSpline3D &guidance_spline = guidance.GetGuidanceTrajectory(0).spline;
            ROS_INFO("[Best Trajectory]");

            RosTools::Spline2D guidance_trajectory = guidance_spline.GetTrajectory(); // Retrieves the trajectory: t -> (x, y))
            for (double t = 0; t < Config::N * Config::DT; t += 4 * Config::DT)
            {
                Eigen::Vector2d pos = guidance_trajectory.getPoint(t);
                ROS_INFO_STREAM("\t[t = " << t << "]: (" << pos(0) << ", " << pos(1) << ")");
            }
            RosTools::Spline2D guidance_path = guidance_spline.GetPath(); // Retrieves the path: s -> (x, y)

            /** @note If you decide on a trajectory to execute, you can provide this feedback to the guidance planner
             *  and it will remember which path is best and prioritize finding it in the next iteration.*/
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

    BENCHMARKERS.print();
    RosTools::Instrumentor::Get().EndSession();
    return 1;
}
