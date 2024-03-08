#ifndef GUIDANCE_PLANNER_RECONFIGURE_H
#define GUIDANCE_PLANNER_RECONFIGURE_H

#ifdef MPC_PLANNER_ROS
#include <dynamic_reconfigure/server.h>
#include <guidance_planner/GuidancePlannerConfig.h> // Included to define the reconfigure callback

namespace GuidancePlanner
{
    class Reconfigure
    {

    public:
        Reconfigure()
        {
            first_reconfigure_callback_ = true;
            ros::NodeHandle nh_guidance("guidance_planner");
            reconfigure_server_.reset(new dynamic_reconfigure::Server<GuidancePlanner::GuidancePlannerConfig>(reconfig_mutex_, nh_guidance));
            reconfigure_server_->setCallback(boost::bind(&GlobalGuidance::ReconfigureCallback, this, _1, _2));
        }

        /** @brief Add some of the settings to the rqt_reconfigure window */
        void ReconfigureCallback(GuidancePlannerConfig &config, uint32_t level)
        {
            if (first_reconfigure_callback_) // Set the reconfiguration parameters to match the yaml configuration at startup
            {
                first_reconfigure_callback_ = false;

                config.debug = Config::debug_output_;

                config.n_paths = config_->n_paths_;
                config.n_samples = config_->n_samples_;
                config.visualize_samples = config_->visualize_all_samples_;
                config.use_learning = config_->use_learning;
            }

            Config::debug_output_ = config.debug;

            config_->n_paths_ = config.n_paths;
            config_->n_samples_ = config.n_samples;
            config_->visualize_all_samples_ = config.visualize_samples;
            config_->use_learning = config.use_learning;
        }

    private:
        bool first_reconfigure_callback_;

        boost::shared_ptr<dynamic_reconfigure::Server<GuidancePlannerConfig>> reconfigure_server_;
        boost::recursive_mutex reconfig_mutex_;
    };
}

#else
namespace GuidancePlanner
{
    class Reconfigure
    {
    };
}
#endif

#endif // GUIDANCE_PLANNER_RECONFIGURE_H
