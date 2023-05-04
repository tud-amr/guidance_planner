#include <fstream>
#include <iostream>
#include <vector>

#include <ros_tools/types.h>

struct PointsList
{
    std::vector<double> x;
    std::vector<double> y;
};

void ReadFile(const std::string &file_name, GuidancePlanner::GlobalGuidance &guidance, std::vector<GuidancePlanner::Obstacle> &obstacles)
{
    obstacles.clear();
    // std::cout << "/home/r2c1/ros/sandbox_ws/src/guidance_planner/debug/10_ts/" + file_name << std::endl;
    std::ifstream input("/home/r2c1/ros/sandbox_ws/src/guidance_planner/debug/20_ts/" + file_name, std::ifstream::binary);
    if (!input)
    {
        std::cout << "file could not be read\n"
                  << std::endl;
        return;
    }

    std::vector<PointsList> obstacles_vec;
    std::vector<Eigen::Vector2d> predictions;
    PointsList goals, human_truth;
    double x, y, orientation, v;

    // Obstacles
    int length, length_obs;
    double d_aux;
    input.read(reinterpret_cast<char *>(&length), sizeof(int));

    for (int i = 0; i < length; i++)
    {
        predictions.clear();
        PointsList obs;
        input.read(reinterpret_cast<char *>(&length_obs), sizeof(length_obs));
        // std::cout << "Obstacle " << i << " with length " << length_obs << ": " << std::endl;
        for (int j = 0; j < length_obs; j++)
        {
            input.read(reinterpret_cast<char *>(&d_aux), sizeof(d_aux));
            obs.x.push_back(d_aux);
            // std::cout << d_aux  << " || " << std::flush;
        }
        for (int j = 0; j < length_obs; j++)
        {
            input.read(reinterpret_cast<char *>(&d_aux), sizeof(d_aux));
            obs.y.push_back(d_aux);
            // std::cout << d_aux  << " || " << std::flush;
        }
        // std::cout << std::endl;
        obstacles_vec.push_back(obs);

        for (int l = 0; l < obstacles_vec.back().x.size(); l++)
        {
            predictions.emplace_back(obstacles_vec.back().x[l], obstacles_vec.back().y[l]);
        }

        obstacles.emplace_back(i, predictions, 0.5);
    }

    /** @brief Static obstacles: Ax <= b */
    std::vector<RosTools::Halfspace> static_obstacles;
    guidance.LoadObstacles(obstacles, static_obstacles);

    // x
    input.read(reinterpret_cast<char *>(&x), sizeof(x));
    // std::cout << "x: " << x << " || " << std::flush;
    // y
    input.read(reinterpret_cast<char *>(&y), sizeof(y));
    // std::cout << "y: " << y << " || " << std::flush;
    // orientation
    input.read(reinterpret_cast<char *>(&orientation), sizeof(orientation));
    // std::cout << "orientation: " << orientation << " || " << std::flush;
    // v
    input.read(reinterpret_cast<char *>(&v), sizeof(v));
    // std::cout << "v: " << v << std::endl;
    guidance.SetStart(Eigen::Vector2d(x, y), orientation, v);

    // goals
    std::vector<GuidancePlanner::Goal> goal_vec;

    input.read(reinterpret_cast<char *>(&length), sizeof(length));
    // std::cout << "length of goals: " << length << std::endl;
    // std::cout << "Goals: " << std::endl;
    for (int i = 0; i < length; i++)
    {
        input.read(reinterpret_cast<char *>(&d_aux), sizeof(d_aux));
        // std::cout << d_aux << std::endl;
        goals.x.push_back(d_aux);
        // std::cout << d_aux  << " || " << std::flush;
    }
    for (int i = 0; i < length; i++)
    {
        input.read(reinterpret_cast<char *>(&d_aux), sizeof(d_aux));
        goals.y.push_back(d_aux);
        // std::cout << d_aux  << " || " << std::flush;
    }

    for (int i = 0; i < length; i++)
        goal_vec.emplace_back(Eigen::Vector2d(goals.x[i], goals.y[i]), 1.);

    guidance.SetGoals(goal_vec);
    guidance.Update();

    guidance.Visualize();
    // guidance.Reset();

    // human truth
    input.read(reinterpret_cast<char *>(&length), sizeof(length));
    for (int i = 0; i < length; i++)
    {
        input.read(reinterpret_cast<char *>(&d_aux), sizeof(d_aux));
        human_truth.x.push_back(d_aux);
        // std::cout << d_aux << " || " << std::flush;
    }
    for (int i = 0; i < length; i++)
    {
        input.read(reinterpret_cast<char *>(&d_aux), sizeof(d_aux));
        human_truth.y.push_back(d_aux);
        // std::cout << d_aux << " || " << std::flush;
    }
    // std::cout << std::endl << "Scenario read" << std::endl;
}