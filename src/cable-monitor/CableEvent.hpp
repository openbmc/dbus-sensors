#pragma once

#include <sdbusplus/async.hpp>

namespace phosphor::cable::event
{

class Event
{
  public:
    /** @brief Event type */
    enum class Type
    {
        connected,
        disconnected,
        unknown
    };

    Event() = delete;
    explicit Event(sdbusplus::async::context& ctx) : ctx(ctx) {}

    /** @brief Generate a cable event */
    auto generateCableEvent(Type type, std::string name)
        -> sdbusplus::async::task<>;

  private:
    /** @brief Resolve a leak event object path */
    auto resolveLeakEvent(sdbusplus::message::object_path eventPath)
        -> sdbusplus::async::task<>;

    /** @brief Map type for event name to log event object path */
    using event_map_t =
        std::unordered_map<std::string, sdbusplus::message::object_path>;

    /** @brief D-Bus context */
    sdbusplus::async::context& ctx;
    /** @brief Map of pending active events */
    event_map_t pendingEvents;
};

} // namespace phosphor::cable::event
