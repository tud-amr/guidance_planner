#include <rclcpp/rclcpp.hpp>

#include <guidance_planner/global_guidance.h>

#include <ros_tools/ros2_wrappers.h>
#include <ros_tools/visuals.h>
#include <ros_tools/profiling.h>

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
    obstacles.clear();
    RosTools::RandomGenerator obstacle_randomizer;
    for (int i = 0; i < 6; i++)
    {
        Eigen::Vector2d start(2.0 + obstacle_randomizer.Double() * 6.0, -4. + 8. * obstacle_randomizer.Double());
        Eigen::Vector2d goal(2.0 + obstacle_randomizer.Double() * 6.0, -4. + 8. * obstacle_randomizer.Double());
        Eigen::Vector2d vel = (goal - start).normalized();

        obstacles.emplace_back(i, start, vel, Config::DT, Config::N, 0.5);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("guidance_example");

    STATIC_NODE_POINTER.init(node.get());
    VISUALS.init(node.get());

    // Initialize profiling
    RosTools::Instrumentor::Get().BeginSession("guidance_planner");

    RCLCPP_INFO(node->get_logger(), "Starting Guidance Planner Example");

    GlobalGuidance guidance;        // Initializes the guidance planner
    config_ = guidance.GetConfig(); // Retrieves the configuration file, if you need it

    // Some settings for the example only
    node->declare_parameter("replan", false);
    node->declare_parameter("example_rate", 1.);

    /** @brief Load inputs for the guidance planner (you should do this for each computation) */
    RCLCPP_INFO(node->get_logger(), "Preparing Obstacles");
    std::vector<Obstacle> obstacles;

    /** @brief Static obstacles: Ax <= b */
    std::vector<GuidancePlanner::Halfspace> static_obstacles;
    static_obstacles.emplace_back(Eigen::Vector2d(0., 1.), 10.);  // y <= 10 (corridor)
    static_obstacles.emplace_back(Eigen::Vector2d(0., -1.), 10.); // y >= -10 (corridor)

    /** @brief Set the robot position */
    guidance.SetStart(Eigen::Vector2d(0., 0.), 0., 2.); // Position, yaw angle, velocity magnitude

    /** @brief Set the goals for the planner */
    bool use_reference_path = true;
    if (use_reference_path)
    {
        std::vector<double> xx, yy;
        xx = {0., 2., 4., 6., 8., 10.};
        yy = {0., 0., 0., 0., 0., 0.};
        std::shared_ptr<RosTools::Spline2D> reference_path = std::make_shared<RosTools::Spline2D>(xx, yy);

        double distance_on_spline = 0.;
        double road_width = 6.;
        guidance.LoadReferencePath(distance_on_spline, reference_path, road_width);
    }
    else
    {
        // Using explicit goals
        std::vector<Goal> goals;
        goals.emplace_back(Eigen::Vector2d(6., 0.), 1.); // x = 6, y = 0, cost = 1
        goals.emplace_back(Eigen::Vector2d(8., 0.), 1.); // x = 8, y = 0, cost = 1
        guidance.SetGoals(goals);
    }

    // ----------------------------
    /** @brief Mimic a control loop */
    rclcpp::Rate rate(node->get_parameter("example_rate").as_double());
    while (rclcpp::ok())
    {
        RCLCPP_WARN(node->get_logger(), "Updating Guidance");
        if (obstacle_source == "Random")
            RandomizeObstacles(obstacles);
        else
            ManualObstacles(obstacles);

        rclcpp::spin_some(node);

        // Normally: load the start and goals in each timestep
        // Then, load the obstacles
        guidance.LoadObstacles(obstacles, static_obstacles);

        // Update (i.e., compute) the guidance trajectories
        guidance.Update();

        // Show some results:
        bool success = guidance.Succeeded();
        if (success)
        {
            RCLCPP_INFO_STREAM(node->get_logger(), "Guidance planner found: " << guidance.NumberOfGuidanceTrajectories() << " trajectories");
            CubicSpline3D &guidance_spline = guidance.GetGuidanceTrajectory(0).spline;
            RCLCPP_INFO(node->get_logger(), "[Best Trajectory]");

            RosTools::Spline2D guidance_trajectory = guidance_spline.GetTrajectory(); // Retrieves the trajectory: t -> (x, y))
            for (double t = 0; t < Config::N * Config::DT; t += 4 * Config::DT)
            {
                Eigen::Vector2d pos = guidance_trajectory.getPoint(t);
                RCLCPP_INFO_STREAM(node->get_logger(), "\t[t = " << t << "]: (" << pos(0) << ", " << pos(1) << ")");
            }
            RosTools::Spline2D guidance_path = guidance_spline.GetPath(); // Retrieves the path: s -> (x, y)

            /** @note If you decide on a used path, you can provide this feedback to the guidance planner and it will remember which path is best */
            // int used_trajectory_id = guidance.GetUsedTrajectory()
            // guidance.SetUsedTrajectory(int spline_id);
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "\tGuidance planner found no trajectories that reach any of the goals!");
        }

        replan_ = false;

        // Keep visualizing until replan is set
        while (rclcpp::ok() && !replan_)
        {
            replan_ = node->get_parameter("replan").as_bool();

            guidance.Visualize(); // Visualize the result in RVIZ
            if (config_->debug_continuous_replanning_)
                replan_ = true;

            rclcpp::spin_some(node);
            rate.sleep();
        }
        node->set_parameter(rclcpp::Parameter("replan", false));
    }

    RosTools::Instrumentor::Get().EndSession();

    return 1;
}