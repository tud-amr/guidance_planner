#ifndef GUIDANCE_PLANNER_UTILS_H
#define GUIDANCE_PLANNER_UTILS_H

#include <ros_tools/logging.h>
#include "gsl/gsl_errno.h"

/** Logging definitions */
// #define PRM_LOGGING_ENABLED 1
// #if PRM_LOGGING_ENABLED == 1
#define PRM_LOG(msg)                \
    if (Config::debug_output_)      \
    {                               \
        LOG_INFO("[PRM]: " << msg); \
    }
// #else
// #define PRM_LOG(msg)
// #endif

#define PRM_WARN(msg) LOG_WARN("[PRM]: " << msg)

// #define PRM_DEBUG_ALL 0
// #if PRM_DEBUG_ALL == 1
#define IF_PRM_DEBUG(x) x
// #else
// #define IF_PRM_DEBUG(x)
// #endif

class IntegrationException : public std::exception
{
public:
    char *what()
    {
        return (char *)"GSL integration Exception";
    }
};

inline void IntegrationExceptionHandler(const char *reason, const char *file, int line, int err)
{
    (void)err;
    gsl_stream_printf("ERROR", file, line, reason);
    fflush(stdout);
    fprintf(stderr, "Customized GSL error handler invoked.\n");
    fflush(stderr);

    throw IntegrationException();
}

#endif // GUIDANCE_PLANNER_UTILS_H
