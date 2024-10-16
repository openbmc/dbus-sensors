#pragma once

#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/client.hpp>

#include <string>
#include <unordered_map>

namespace leak
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

    sdbusplus::async::task<> generateLeakEvent(
        sdbusplus::message::object_path detectorPath,
        DetectorIntf::DetectorState state,
        detector::config::DetectorLevel level);

  private:
    sdbusplus::async::task<> resolveLeakEvent(
        sdbusplus::message::object_path eventPath);

    /** @brief Map type for event name to log event object path */
    using event_map_t =
        std::unordered_map<std::string, sdbusplus::message::object_path>;

    sdbusplus::async::context& ctx;
    event_map_t pendingEvents;
};

} // namespace events

} // namespace leak
