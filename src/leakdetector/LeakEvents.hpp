#pragma once

#include <sdbusplus/async.hpp>
#include <xyz/openbmc_project/Configuration/GPIOLeakDetector/aserver.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/aserver.hpp>

#include <string>

namespace phosphor::leak::events
{

using DetectorIntf =
    sdbusplus::common::xyz::openbmc_project::state::leak::Detector;
using DetectorConfigIntf =
    sdbusplus::common::xyz::openbmc_project::configuration::GPIOLeakDetector;

class LeakEvents
{
  public:
    LeakEvents() = delete;

    explicit LeakEvents(sdbusplus::async::context& ctx) : ctx(ctx) {}

    /** @brief Generate a leak event */
    auto generateLeakEvent(
        const sdbusplus::message::object_path& detectorPath,
        DetectorIntf::DetectorState state,
        DetectorConfigIntf::DetectorLevel level) -> sdbusplus::async::task<>;

  private:
    /** @brief Resolve a leak event for a given object path */
    auto resolveLeakEvent(sdbusplus::message::object_path& eventPath)
        -> sdbusplus::async::task<>;

    /** @brief Map type for event name to log event object path */
    using event_map_t =
        std::unordered_map<std::string, sdbusplus::message::object_path>;

    /** @brief D-Bus context */
    sdbusplus::async::context& ctx;
    /** @brief Map of pending active events */
    event_map_t pendingEvents;
};

} // namespace phosphor::leak::events
