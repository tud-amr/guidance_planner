#ifndef __CONNECTION_H__
#define __CONNECTION_H__

#include <guidance_planner/types/node.h>

#include <guidance_planner/third_party/dubins.h>

namespace GuidancePlanner
{

    /** @brief Connect two points with a steering function */
    class Connection
    {
    public:
        Connection(Node *a, Node *b);

        Connection(const Connection &other)
        {
            a_ = other.getStart();
            b_ = other.getEnd();
        }

        // Define the copy assignment operator
        Connection &operator=(const Connection &other)
        {
            if (this != &other)
            {
                // Copy assign each member appropriately
                this->a_ = other.getStart();
                this->b_ = other.getEnd();
                // Add copying logic for other members
            }
            return *this;
        }

    public:
        virtual SpaceTimePoint operator()(double s) const = 0;

        virtual bool isValid(Config *config, double orientation) const;
        virtual void getIntegrationNodes(bool is_first, std::vector<SpaceTimePoint> &nodes) const;

        virtual double length() const = 0;
        virtual double lengthWithTime() const = 0;

        Node *getStart() const { return a_; }
        Node *getEnd() const { return b_; }

    protected:
        Node *a_;
        Node *b_;
    };

    class StraightConnection : public Connection
    {
    public:
        StraightConnection(Node *a, Node *b);

        StraightConnection(const StraightConnection &other)
            : Connection(other){};

        SpaceTimePoint operator()(double s) const override;
        double length() const override;
        double lengthWithTime() const override;
    };

    class DubinsConnection : public Connection
    {
    public:
        DubinsConnection(Node *a, Node *b);

        DubinsConnection(const DubinsConnection &other) = default;

    public:
        SpaceTimePoint operator()(double s) const override;
        double length() const override;
        double lengthWithTime() const override;

        virtual bool isValid(Config *config, double orientation) const override;
        virtual void getIntegrationNodes(bool is_first, std::vector<SpaceTimePoint> &nodes) const override;

    private:
        DubinsPath path_;
        std::vector<SpaceTimePoint> sampled_path_;

        bool valid_{false};
        double length_{0.0};
        double length_with_time_{0.0};
    };
} // namespace GuidancePlanner

#endif // __CONNECTION_H__