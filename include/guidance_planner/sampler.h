#ifndef GUIDANCE_PLANNER_SAMPLER_H
#define GUIDANCE_PLANNER_SAMPLER_H

#include <guidance_planner/config.h>

#include <guidance_planner/types/types.h>

#include <ros_tools/random_generator.h>
#include <ros_tools/spline.h>

#include <vector>
#include <string>

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
        // using SampleFunction = Sample &(Sampler::*)(int);
        using SampleFunction = std::function<Sample &(int)>;

        void SetRange(const SpaceTimePoint::PVector &start, const std::vector<Goal> &goals);

        void SetSampleMethod(const std::string &&method);

        void SampleAlongReferencePath(std::shared_ptr<RosTools::Spline2D> reference_path,
                                      double cur_s, double max_s, double width);

        Sample &DrawSample(int sample_index);

        Sample &GetSample(int sample_index)
        {
            return samples_[sample_index];
        }

        void Clear();
        void Reset();

        void Visualize() const;

    private:
        Config *config_;

        RosTools::RandomGenerator random_generator_; // Used to generate samples

        SpaceTimePoint::PVector min_, max_, range_;

        std::vector<Sample> samples_;

        SampleFunction sample_function_ptr_;

        Sample &SampleUniformly(int sample_index);
        Sample &SampleUniformlyWithOrientation(int sample_index);
        Sample &SampleAlongPath(int sample_index,
                                std::shared_ptr<RosTools::Spline2D> reference_path,
                                double min_s, double range_s,
                                double min_lat, double range_lat);
    };

} // namespace GuidancePlanner

#endif // GUIDANCE_PLANNER_SAMPLER_H
