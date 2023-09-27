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

#include <ros_tools/helpers.h>

#include <geometry_msgs/Pose.h>
#include <jsk_rviz_plugins/OverlayText.h>

#include <boost/bind.hpp>

#include <vector>
#include <Eigen/Dense>
#include <unordered_map>

namespace GuidancePlanner
{

    struct ColorManager
    {

        std::vector<bool> color_idxs;

        ColorManager(int size)
        {
            Reset(size);
        }

        void Reset(int size) { color_idxs = std::vector<bool>(size, false); }

        void ReleaseColor(int idx)
        {
            color_idxs[idx] = false;
        }

        /**
         * @brief Try to claim a color (return false if already claimed)
         *
         * @param idx Index of color
         * @return true If color is available
         * @return false If color is claimed
         */
        bool ClaimColor(int idx)
        {
            if (color_idxs[idx])
                return false;

            color_idxs[idx] = true;
            return true;
        }

        int GetColor()
        {
            for (size_t i = 0; i < color_idxs.size(); i++)
            {
                if (!color_idxs[i])
                {
                    color_idxs[i] = true;
                    return i;
                }
            }

            throw std::runtime_error("No colors available (too many trajectories!)");
        }
    };

// To distinguish custom from regular optimization loops.
#define EXIT_CODE_NOT_OPTIMIZED_YET -999

    // MINOR TYPES
    struct Path
    {
        std::vector<double> x_;
        std::vector<double> y_;
        std::vector<double> psi_;

        /**
         * @brief Construct a new Path object
         *
         * @param length Size to allocate (reserve)
         */
        Path(int length = 10)
        {
            x_.reserve(length);
            y_.reserve(length);
            psi_.reserve(length);
        }

        void AddPose(const geometry_msgs::Pose &pose)
        {
            x_.push_back(pose.position.x);
            y_.push_back(pose.position.y);
            psi_.push_back(RosTools::quaternionToAngle(pose));
        }

        void Clear()
        {
            x_.clear();
            y_.clear();
            psi_.clear();
        }

        friend std::ostream &operator<<(std::ostream &out, const Path &path)
        {

            out << "Path:\n";
            for (size_t i = 0; i < path.x_.size(); i++)
            {
                out << "(" << path.x_[i] << ", " << path.y_[i] << ", " << path.psi_[i] << ")" << std::endl;
            }
            return out;
        }
    };

    // /** Virtual class for splines in 2D (x, y)*/
    // class Spline2D
    // {
    // public:
    //     virtual Eigen::Vector2d GetPoint(double s) const = 0;
    //     virtual Eigen::Vector2d GetVelocity(double s) const = 0;
    //     virtual Eigen::Vector2d GetAcceleration(double s) const = 0;
    //     virtual void GetParameters(int index,
    //                                double &ax, double &bx, double &cx, double &dx,
    //                                double &ay, double &by, double &cy, double &dy) = 0;

    //     virtual double GetSplineStart(int index) = 0;
    //     virtual double GetSplineEnd(int index) = 0;
    //     virtual double GetSplineLength() = 0;
    //     virtual int NumberOfSegments() = 0;
    //     virtual void Print() = 0;
    // };

    /** @brief Implementation of a cubic spline that can be intialized with a,b,c,d and a distance */
    class CubicSpline
    {
    public:
        std::vector<double> a_, b_, c_, d_;
        std::vector<double> knot_distances_;

        CubicSpline()
        {
            knot_distances_.push_back(0.);
        }

        // Add a spline
        void AddSpline(double a, double b, double c, double d, double L)
        {
            a_.push_back(a);
            b_.push_back(b);
            c_.push_back(c);
            d_.push_back(d);

            knot_distances_.push_back(L);
        }

        // Add a spline
        void AddSpline(const Eigen::Vector4d &abcd, double L)
        {
            a_.push_back(abcd(0));
            b_.push_back(abcd(1));
            c_.push_back(abcd(2));
            d_.push_back(abcd(3));

            knot_distances_.push_back(L);
        }

        // Evaluate the spline at distance s
        double operator()(double s) const
        {
            int spline_idx = -1;
            double spline_start = 0.;
            for (size_t i = 0; i < knot_distances_.size() - 1; i++)
            {

                // Find the correct interval for s, i.e., where s_i <= s < s_i+1 (also use = at the end to include the final point)
                // Beyond the spline, use the last segment
                if ((s >= knot_distances_[i] && s <= knot_distances_[i + 1]) || (i == knot_distances_.size() - 2))
                {
                    spline_idx = i;
                    spline_start = knot_distances_[i];

                    break;
                }
            }

            if (s > knot_distances_.back() || spline_idx == -1)
            {
                ROS_WARN_STREAM("Spline evaluated outside of range (s = " << std::to_string(s) << ", max length = " << std::to_string(knot_distances_.back()) << ") - spline_idx = " << spline_idx);
            }

            double ss = (s - spline_start); // / (knot_distances_[spline_idx + 1] - spline_start); // Normalized in [0, 1]
            return a_[spline_idx] * std::pow(ss, 3.) +
                   b_[spline_idx] * std::pow(ss, 2.) +
                   c_[spline_idx] * ss +
                   d_[spline_idx];
        }

        /** @brief Add interface for retrieving a, b, c, d */
        void GetParameters(int index, double &a, double &b, double &c, double &d)
        {
            if (index >= (int)a_.size())
                ROS_WARN_STREAM("GetParameters: Accessed parameter beyond cubic spline definition (accessed: " << index << ", size: " << a_.size() << ")");

            a = a_[index];
            b = b_[index];
            c = c_[index];
            d = d_[index];
        }

        double GetSplineStart(int index)
        {
            // Prevent any index higher than the second to last
            index = std::min(index, (int)knot_distances_.size() - 2);

            return knot_distances_[index];
        }

        double GetSplineEnd(int index)
        {
            if (index >= (int)knot_distances_.size() - 1)
                ROS_WARN_STREAM("GetDistance: Accessed parameter beyond cubic spline definition (accessed: " << index << ", size: " << knot_distances_.size() - 1 << ")");

            index = std::min(index, (int)knot_distances_.size() - 1);
            return knot_distances_[index + 1];
        }

        friend std::ostream &operator<<(std::ostream &stream, const CubicSpline &spline)
        {
            stream << "==== CubicSpline ====\n";
            for (size_t i = 0; i < spline.a_.size(); i++)
            {
                stream << "s" << i << " = [" << spline.knot_distances_[i] << ", " << spline.knot_distances_[i + 1] << "]\n"
                       << "coef" << i << " = [" << spline.a_[i] << ", " << spline.b_[i] << ", " << spline.c_[i] << ", " << spline.d_[i] << "]\n";
            }

            return stream;
        }
    };

    // /** Combined definition of 2D cubic splines (x, y), can use multiple types of classes as long as the () operator evaluates the spline*/
    // template <class T>
    // class RosTools::CubicSpline2D : public Spline2D
    // {
    // public:
    //     RosTools::CubicSpline2D(const T &spline_x, const T &spline_y)
    //         : spline_x_(spline_x), spline_y_(spline_y)
    //     {
    //     }

    //     RosTools::CubicSpline2D(const RosTools::CubicSpline2D<T> &other) // copy constructor
    //     {
    //         spline_x_ = other.spline_x_;
    //         spline_y_ = other.spline_y_;
    //     }

    //     Eigen::Vector2d GetPoint(double s) const override
    //     {
    //         return Eigen::Vector2d(
    //             spline_x_(s),
    //             spline_y_(s));
    //     }

    //     Eigen::Vector2d GetVelocity(double s) const override
    //     {
    //         return Eigen::Vector2d(
    //             spline_x_.deriv(1, s),
    //             spline_y_.deriv(1, s));
    //     }

    //     Eigen::Vector2d GetAcceleration(double s) const override
    //     {
    //         return Eigen::Vector2d(
    //             spline_x_.deriv(2, s),
    //             spline_y_.deriv(2, s));
    //     }

    //     void GetParameters(int index,
    //                        double &ax, double &bx, double &cx, double &dx,
    //                        double &ay, double &by, double &cy, double &dy) override
    //     {
    //         spline_x_.GetParameters(index, ax, bx, cx, dx);
    //         spline_y_.GetParameters(index, ay, by, cy, dy);
    //     }

    //     double GetSplineStart(int index) override
    //     {
    //         return spline_x_.GetSplineStart(index);
    //     }

    //     double GetSplineEnd(int index) override
    //     {
    //         return spline_x_.GetSplineEnd(index);
    //     }

    //     double GetSplineLength() override
    //     {
    //         return spline_x_.m_x_.back();
    //     }

    //     int NumberOfSegments() override
    //     {
    //         return spline_x_.m_a.size() - 1; // Don't use the last segment (it is an extrapolation to the right)
    //     }

    //     void Print() override
    //     {
    //         spline_x_.Print();
    //         spline_y_.Print();
    //     }

    // private:
    //     T spline_x_, spline_y_;
    // };

    // struct RosTools::Halfspace
    // {
    //     Eigen::Vector2d A_;
    //     double b_;

    //     RosTools::Halfspace(){};

    //     RosTools::Halfspace(const Eigen::Vector2d &A, const double b)
    //         : A_(A), b_(b)
    //     {
    //     }

    //     static RosTools::Halfspace &Dummy()
    //     {
    //         static RosTools::Halfspace dummy(Eigen::Vector2d(1, 0), 1000);
    //         return dummy;
    //     }
    // };

    /** @todo Simpler monitor class */
    class Monitor
    {
    public:
        Monitor(ros::NodeHandle &nh)
        {
            text_publisher_ = nh.advertise<jsk_rviz_plugins::OverlayText>("/lmpcc/interface_status", 1);
        };

    public:
        /** @brief Mark that the callback with name "name" is expected to be called in each iteration. If only once is enough, set persistent to true */
        void MarkExpected(const std::string &&name, bool persistent = false)
        {
            received_[name] = false; // Adds it to the map

            if (persistent)
                persistent_[name] = true;
        }

        void MarkReceived(const std::string &&name)
        {
            received_[name] = true;
        }

        // Print the status of all the signals and reset
        void PrintStatus()
        {
            std::string message = "Signal Status:\n";
            for (auto &signal : received_)
            {
                if (signal.second) // Check the boolean to which this signal maps (i.e., the received flag)
                {
                    message += "\t" + std::string(signal.first) + ": Received\n";
                    if (persistent_.find(signal.first) == persistent_.end()) // If this variable is not persistent, reset its value to not received
                        signal.second = false;
                }
                else
                    message += "\t" + std::string(signal.first) + ": MISSING\n";
            }

            // Show a message on the screen
            jsk_rviz_plugins::OverlayText overlay_msg;
            overlay_msg.text = message;
            overlay_msg.action = 0;
            overlay_msg.width = 100;
            overlay_msg.height = 150;
            overlay_msg.left = 10;
            overlay_msg.top = 80;
            overlay_msg.bg_color.a = 0.2;
            overlay_msg.line_width = 2;
            overlay_msg.text_size = 12.0;
            overlay_msg.font = "DejaVu Sans Mono";
            overlay_msg.fg_color.r = 0.098;
            overlay_msg.fg_color.g = 0.94;
            overlay_msg.fg_color.b = 0.94;
            overlay_msg.fg_color.a = 1.0;

            text_publisher_.publish(overlay_msg);
        }

    private:
        std::map<std::string, bool> received_;
        std::map<std::string, bool> persistent_;

        ros::Publisher text_publisher_;
    };

};
#endif // __GUIDANCE_TYPES_H__