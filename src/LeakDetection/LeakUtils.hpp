#pragma once

#include <phosphor-logging/lg2.hpp>

#include <string>

PHOSPHOR_LOG2_USING;

constexpr auto serviceName = "xyz.openbmc_project.LeakDetector";

namespace phosphor::leak::utils
{
enum class Level
{
    warning,
    critical,
    unknown,
};

void generateLeakEvent(std::string& name, Level level);

void resolveLeakEvent(std::string& name, Level level);

} // namespace phosphor::leak::utils
