#pragma once

#include <sdbusplus/async.hpp>

#include <string>

constexpr auto serviceName = "xyz.openbmc_project.LeakDetector";

namespace phosphor::leak::utils
{

/**@brief Severity level for leak event */
enum class Level
{
    warning,
    critical,
    unknown,
};

/** @brief Generate a leak event */
void generateLeakEvent(sdbusplus::async::context& ctx, const std::string& name,
                       bool state, Level level);

} // namespace phosphor::leak::utils
