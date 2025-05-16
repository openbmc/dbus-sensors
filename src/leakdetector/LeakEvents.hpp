#pragma once

#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/client.hpp>

#include <string>
#include <unordered_map>

namespace leak
{

namespace config
{
enum class DetectorLevel;
}

using DetectorStateIntf =
    sdbusplus::client::xyz::openbmc_project::state::leak::Detector<>;

class Events
{
  public:
    Events() = delete;

    explicit Events(sdbusplus::async::context& ctx) : ctx(ctx) {}

    sdbusplus::async::task<> generateLeakEvent(
        sdbusplus::message::object_path detectorPath,
        DetectorStateIntf::DetectorState state, config::DetectorLevel level);

  private:
    /** @brief Map type for event name to log event object path */
    using event_map_t =
        std::unordered_map<std::string, sdbusplus::message::object_path>;

    sdbusplus::async::context& ctx;
    event_map_t pendingEvents;
};

} // namespace leak
