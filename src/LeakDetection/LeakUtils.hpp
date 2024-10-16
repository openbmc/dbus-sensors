#pragma once

#include <sdbusplus/async.hpp>
#include <xyz/openbmc_project/Configuration/GPIOLeakDetector/aserver.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/aserver.hpp>

#include <string>

constexpr auto serviceName = "xyz.openbmc_project.LeakDetector";

namespace phosphor::leak::utils
{

using DetectorIntf =
    sdbusplus::common::xyz::openbmc_project::state::leak::Detector;
using DetectorConfigIntf =
    sdbusplus::common::xyz::openbmc_project::configuration::GPIOLeakDetector;

/** @brief Generate a leak event */
void generateLeakEvent(sdbusplus::async::context& ctx, const std::string& name,
                       DetectorIntf::DetectorState state,
                       DetectorConfigIntf::DetectorLevel level);

} // namespace phosphor::leak::utils
