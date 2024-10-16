#pragma once

#include <sdbusplus/async.hpp>
#include <xyz/openbmc_project/Configuration/GPIOLeakDetector/aserver.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/client.hpp>

#include <string>

namespace phosphor::leak
{

namespace detector::config
{
enum class DetectorLevel;
}

namespace events
{

using DetectorIntf =
    sdbusplus::client::xyz::openbmc_project::state::leak::Detector<>;

class LeakEvents
{
  public:
    LeakEvents() = delete;

    explicit LeakEvents(sdbusplus::async::context& ctx) : ctx(ctx) {}

    auto generateLeakEvent(
        sdbusplus::message::object_path detectorPath,
        DetectorIntf::DetectorState state,
        detector::config::DetectorLevel level) -> sdbusplus::async::task<>;

  private:
    auto resolveLeakEvent(sdbusplus::message::object_path eventPath)
        -> sdbusplus::async::task<>;

    /** @brief Map type for event name to log event object path */
    using event_map_t =
        std::unordered_map<std::string, sdbusplus::message::object_path>;

    sdbusplus::async::context& ctx;
    event_map_t pendingEvents;
};

} // namespace events

} // namespace phosphor::leak
