#include <guidance_planner/sampler.h>

#include <ros_tools/logging.h>
#include <ros_tools/visuals.h>

#include <functional>

namespace GuidancePlanner
{
    Sampler::Sampler(Config *config)
        : config_(config)
    {
        samples_.resize(config_->n_samples_);

        min_ = SpaceTimePoint::PVector::Zero();
        max_ = SpaceTimePoint::PVector::Zero();
        range_ = SpaceTimePoint::PVector::Zero();

        random_generator_ = RosTools::RandomGenerator(config_->seed_);

        SetSampleMethod("Uniform");
    }

    void Sampler::SetSampleMethod(const std::string &&method)
    {
        LOG_VALUE("Sampling Method", method);
        if (method == "Uniform")
            sample_function_ptr_ = std::bind(&Sampler::SampleUniformly, this, std::placeholders::_1);
        else if (method == "UniformWithOrientation")
            sample_function_ptr_ = std::bind(&Sampler::SampleUniformlyWithOrientation, this, std::placeholders::_1);
    }

    void Sampler::SampleAlongReferencePath(std::shared_ptr<RosTools::Spline2D> reference_path,
                                           const double cur_s, const double max_s, const double width)
    {
        LOG_INFO_THROTTLE(50000, "Overriding uniform sampling method with sampling along the reference path");
        sample_function_ptr_ = std::bind(&Sampler::SampleAlongPath, this, std::placeholders::_1,
                                         reference_path, cur_s, max_s - cur_s, -width / 2., width);
    }

    void Sampler::SetRange(const SpaceTimePoint::PVector &start, const std::vector<Goal> &goals)
    {
        min_ = start;
        max_ = start;

        // Compute the range of the planning space (between start and goals)
        for (auto &goal : goals)
        {
            for (int i = 0; i < SpaceTimePoint::numPositions(); i++)
            {
                min_(i) = std::min(goal.pos(i), min_(i));
                max_(i) = std::max(goal.pos(i), max_(i));
            }
        }
        min_ -= 0.5 * config_->sample_margin_ * SpaceTimePoint::PVector::Ones();
        max_ += 0.5 * config_->sample_margin_ * SpaceTimePoint::PVector::Ones();

        range_ = max_ - min_;

        if (range_.norm() < 1e-3)
            LOG_WARN("The PRM sampling range is zero!");
    }

    void Sampler::Visualize() const
    {
        if (!config_->visualize_all_samples_)
            return;

        auto &sample_visuals = VISUALS.getPublisher("guidance_planner/samples");
        auto &samples = sample_visuals.getNewPointMarker("SPHERE");
        samples.setScale(.15, .15, .15);
        samples.setColorInt(0);

        for (auto &sample : samples_)
        {
            if (sample.success)
                samples.addPointMarker(sample.point.MapToTime());
        }
        sample_visuals.publish();
    }

    void Sampler::Reset()
    {
        random_generator_ = RosTools::RandomGenerator(config_->seed_);
    }

    void Sampler::Clear()
    {
        samples_.clear();
        samples_.resize(config_->n_samples_);
    }

    Sample &Sampler::DrawSample(int sample_index)
    {
        return sample_function_ptr_(sample_index);
    }

    Sample &Sampler::SampleUniformly(int sample_index)
    {
        Sample &sample = samples_[sample_index];

        // Sample positions uniformly
        for (int i = 0; i < SpaceTimePoint::numPositions(); i++)
            sample.point(i) = min_(i) + random_generator_.Double() * range_(i);

        // Sample time [DISCRETE TIME]
        sample.point.SetTime(random_generator_.Int(Config::N - 2) + 1);

        return sample;
    }

    Sample &Sampler::SampleAlongPath(int sample_index,
                                     std::shared_ptr<RosTools::Spline2D> reference_path,
                                     double min_s, double range_s,
                                     double min_lat, double range_lat)
    {
        if (reference_path == nullptr)
        {
            LOG_WARN("Reference path not supplied for sampling");
            return SampleUniformly(sample_index);
        }
        Sample &sample = samples_[sample_index];

        // Sample along the longitudinal and lateral position on the path
        double s = min_s + random_generator_.Double() * range_s;
        double y_dev = min_lat + random_generator_.Double() * range_lat;
        auto &point = reference_path->getPoint(s) + reference_path->getOrthogonal(s) * y_dev;

        sample.point = SpaceTimePoint(point(0), point(1),
                                      random_generator_.Int(Config::N - 2) + 1);

        return sample;
    }

    Sample &Sampler::SampleUniformlyWithOrientation(int sample_index)
    {
        SampleUniformly(sample_index);
        Sample &sample = samples_[sample_index];

        int orientation_index = SpaceTimePoint::numPositions(); // Assume orientation comes after the position
        sample.point(orientation_index) = 0.785398 - 0.5 * M_PI + M_PI * random_generator_.Double();
        return sample;
    }

}