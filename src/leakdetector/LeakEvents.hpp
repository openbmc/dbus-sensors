#pragma once

#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/client.hpp>

#include <map>
#include <string>
#include <tuple>

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

    auto generateLeakEvent(sdbusplus::message::object_path detectorPath,
                           DetectorStateIntf::DetectorState state,
                           config::DetectorLevel level)
        -> sdbusplus::async::task<>;

  private:
    /** @brief Map type for event name to log event object path */
    using event_key_t = std::tuple<std::string, config::DetectorLevel>;
    using event_map_t = std::map<event_key_t, sdbusplus::message::object_path>;

    sdbusplus::async::context& ctx;
    event_map_t pendingEvents;
};

} // namespace leak
