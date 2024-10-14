#include <guidance_planner/types/connection.h>

#include <guidance_planner/utils.h>

#include <ros_tools/math.h>

namespace GuidancePlanner
{
    Connection::Connection(Node *a, Node *b)
    {
        a_ = a;
        b_ = b;
    }

    bool Connection::isValid(Config *config, double orientation) const
    {
        // Check if the connection moves forward in the direction of the path
        if (config->enable_forward_filter_)
        {
            // Check if the vector from a to b is in the direction of the robot
            Eigen::Vector2d vec_ab = b_->point_.Pos() - a_->point_.Pos();
            Eigen::Vector2d vec_path(std::cos(orientation), std::sin(orientation));

            vec_ab.normalize();
            vec_path.normalize();
            double dot_product = vec_ab.dot(vec_path);

            bool forward_connection = dot_product >= 0.0; // This means angle <= 90 degrees

            if (!forward_connection)
            {
                PRM_LOG("Connection does not move forward in time");
                return false;
            }
        }

        // Check if the connection velocity is small enough
        if (config->enable_velocity_filter_)
        {

            double delta_dist = (a_->point_.Pos() - b_->point_.Pos()).norm();
            double delta_time = std::abs(a_->point_.Time() - b_->point_.Time()) * Config::DT;
            double vel = delta_dist / delta_time;
            bool vel_satisfies_limits = vel < config->max_velocity_;

            if (!vel_satisfies_limits)
            {
                PRM_LOG("Connection does not satisfy velocity limits");
                return false;
            }
        }

        return true; // Connection is valid
    }

    void Connection::getIntegrationNodes(bool is_first, std::vector<SpaceTimePoint> &nodes) const
    {
        if (is_first)
            nodes.push_back(a_->point_);

        nodes.push_back(b_->point_);
    }

    // STRAIGHT //

    StraightConnection::StraightConnection(Node *a, Node *b) : Connection(a, b) {}

    SpaceTimePoint StraightConnection::operator()(double s) const // [0-1] -> x
    {
        ROSTOOLS_ASSERT(s >= 0. - 1e-3 && s <= 1. + 1e-3, "StraightConnection only accepts s in [0, 1]");

        return RosTools::InterpolateLinearly(0., 1., s, a_->point_, b_->point_);
    };

    double StraightConnection::length() const
    {
        return (a_->point_.Pos() - b_->point_.Pos()).norm();
    }

    double StraightConnection::lengthWithTime() const
    {
        return (a_->point_.MapToTime() - b_->point_.MapToTime()).norm();
    }

    // DUBINS //

    DubinsConnection::DubinsConnection(Node *a, Node *b) : Connection(a, b)
    {
        double angle_a = 0.; // Could use the start orientation
        double angle_b = 0.;
        if (a_->point_.numStates() == 3)
        {
            angle_a = a->point_.State()(2);
            angle_b = b->point_.State()(2);
        }

        double q0[] = {a->point_.Pos()(0), a->point_.Pos()(1), angle_a};
        double q1[] = {b->point_.Pos()(0), b->point_.Pos()(1), angle_b};
        double turning_radius = Config::turning_radius_; // 1.0;

        int result = dubins_shortest_path(&path_, q0, q1, turning_radius);

        valid_ = result == 0;

        if (!valid_)
        {
            LOG_ERROR("Error computing the Dubins path");
            return;
        }

        length_ = dubins_path_length(&path_);

        for (double s = 0.; s <= 1.0; s += 0.05)
        {
            sampled_path_.push_back(this->operator()(s));

            if (s > 0)
                length_with_time_ += (sampled_path_.back().MapToTime() - sampled_path_[sampled_path_.size() - 2].MapToTime()).norm();
        }
    }

    void DubinsConnection::getIntegrationNodes(bool is_first, std::vector<SpaceTimePoint> &nodes) const
    {
        int start = is_first ? 0 : 1;
        for (int i = start; i < sampled_path_.size(); i++)
            nodes.push_back(sampled_path_[i]);
    }

    SpaceTimePoint DubinsConnection::operator()(double s) const // [0-1] -> x
    {
        s *= length_;

        double out[3];
        dubins_path_sample(&path_, s, out);

        double time = RosTools::InterpolateLinearly(0., length_, s, a_->point_.Time(), b_->point_.Time());

        SpaceTimePoint result;

        result(0) = out[0];
        result(1) = out[1];

        if (a_->point_.numStates() == 3)
        {
            result(2) = out[2]; // Orientation
            result(3) = time;
        }
        else
        {
            result(2) = time;
        }

        return result;
    }

    bool DubinsConnection::isValid(Config *config, double orientation) const
    {
        if (!valid_)
            return false;

        return Connection::isValid(config, orientation);
    }

    double DubinsConnection::length() const
    {
        return length_;
    }

    double DubinsConnection::lengthWithTime() const
    {
        return length_with_time_;
    }

} // namespace GuidancePlanner