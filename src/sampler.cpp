#include <guidance_planner/sampler.h>

#include <ros_tools/visuals.h>

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
    }

    void Sampler::SetRange(const SpaceTimePoint::PVector &start, const std::vector<Goal> &goals)
    {
        min_ = start;
        max_ = start;

        // Compute the range of the planning space (between start and goals)
        // double min_x = start(0), max_x = start(0), min_y = start(1), max_y = start(1);
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
        // all_samples_.clear();
        samples_.clear();

        samples_.resize(config_->n_samples_);
    }

    Sample &Sampler::SampleUniformly(int sample_index)
    {

        Sample &sample = samples_[sample_index];

        // Sample positions uniformly
        for (int i = 0; i < SpaceTimePoint::numPositions(); i++)
        {
            sample.point(i) = min_(i) + random_generator_.Double() * range_(i);
        }

        // Sample time [DISCRETE TIME]
        sample.point.setTime(random_generator_.Int(Config::N - 2) + 1);

        return sample;

        // return SpaceTimePoint(min_x_ + random_generator_.Double() * range_x_,
        //                       min_y_ + random_generator_.Double() * range_y_,
        //                       random_generator_.Int(Config::N - 2) + 1);
    }

}