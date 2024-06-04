#ifndef GUIDANCE_PLANNER_SAMPLER_H
#define GUIDANCE_PLANNER_SAMPLER_H

#include <guidance_planner/config.h>

#include <guidance_planner/types/types.h>

#include <ros_tools/random_generator.h>

#include <vector>

namespace GuidancePlanner
{
    struct Sample
    {
        SpaceTimePoint point;
        bool success;

        Sample() : point(), success(true) {}
        Sample(const SpaceTimePoint &p) : point(p), success(true) {}
    };

    class Sampler
    {

    public:
        Sampler(Config *config);

    public:
        void SetRange(const SpaceTimePoint::PVector &start, const std::vector<Goal> &goals);

        Sample &SampleUniformly(int sample_index);
        Sample &SampleUniformlyWithOrientation(int sample_index);

        Sample &GetSample(int sample_index) { return samples_[sample_index]; }

        void Clear();
        void Reset();

        void Visualize() const;

    private:
        Config *config_;

        RosTools::RandomGenerator random_generator_; // Used to generate samples

        SpaceTimePoint::PVector min_, max_, range_;

        std::vector<Sample> samples_;
    };

} // namespace GuidancePlanner

#endif // GUIDANCE_PLANNER_SAMPLER_H
