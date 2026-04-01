#pragma once

#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <map>
#include <string>

namespace valve
{

class Events
{
  public:
    Events() = delete;

    explicit Events(sdbusplus::async::context& ctx) : ctx(ctx) {}

    auto generateValveEvent(sdbusplus::message::object_path valvePath,
                            bool open) -> sdbusplus::async::task<>;

    auto handleValveSetPointWarning(sdbusplus::message::object_path valvePath,
                                    bool asserted) -> sdbusplus::async::task<>;

  private:
    /** @brief Map of valve path to pending event object path */
    using event_map_t = std::map<std::string, sdbusplus::message::object_path>;

    sdbusplus::async::context& ctx;
    event_map_t pendingEvents;
};

} // namespace valve
