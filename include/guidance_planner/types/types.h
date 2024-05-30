/**
 * @file types.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Defines basic classes for use in LMPCC
 * @version 0.1
 * @date 2022-05-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __GUIDANCE_TYPES_H__
#define __GUIDANCE_TYPES_H__

#include <guidance_planner/config.h>

#include <guidance_planner/types/space_time_point.h>
#include <guidance_planner/types/type_define.h>
#include <guidance_planner/types/node.h>

#include <Eigen/Dense>

#include <vector>

namespace GuidancePlanner
{

    struct ColorManager
    {

        std::vector<bool> color_idxs;

        ColorManager(int size);

        void Reset(int size);

        void ReleaseColor(int idx);

        /**
         * @brief Try to claim a color (return false if already claimed)
         *
         * @param idx Index of color
         * @return true If color is available
         * @return false If color is claimed
         */
        bool ClaimColor(int idx);
        int GetColor();
    };

    struct Halfspace
    {
        Eigen::Vector2d A_;
        double b_;

        Halfspace(){};

        Halfspace(const Eigen::Vector2d &A, const double b) : A_(A), b_(b) {}

        static Halfspace &Dummy()
        {
            static Halfspace dummy(Eigen::Vector2d(1, 0), 1000);
            return dummy;
        }
    };

    /** Temporary */
    struct Obstacle
    {

        int id_;
        std::vector<Eigen::Vector2d> positions_;
        double radius_;

        Obstacle(int id, const Eigen::Vector2d &pos, const Eigen::Vector2d &velocity, double dt, int N, double radius);
        Obstacle(int id, const std::vector<Eigen::Vector2d> &positions, double radius);
    };

    struct Goal
    {
        Node *node = nullptr;
        double cost;
        Eigen::Vector2d pos;

        Goal(const Eigen::Vector2d &_pos, const double _cost)
        {

            pos = _pos;
            cost = _cost;
        };

        static const Goal &FindGoalWithNode(const std::vector<Goal> &goals, const Node *node)
        {
            for (auto &goal : goals)
            {
                if (node->id_ == goal.node->id_)
                    return goal;
            }
            throw std::runtime_error("FindGoalWithNode: Goal not found!");
        }
    };
}
#endif // __GUIDANCE_TYPES_H__