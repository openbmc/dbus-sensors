#pragma once

#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <string>
#include <unordered_map>

namespace cable
{

class Events
{
  public:
    /** @brief Event type */
    enum class Type
    {
        connected,
        disconnected,
        unknown
    };

    Events() = delete;
    explicit Events(sdbusplus::async::context& ctx) : ctx(ctx) {}

    /** @brief Generate a cable event */
    sdbusplus::async::task<> generateCableEvent(Type type, std::string name);

  private:
    /** @brief Resolve a cable event object path */
    sdbusplus::async::task<> resolveCableEvent(
        sdbusplus::message::object_path eventPath);

    /** @brief Map type for event name to log event object path */
    using event_map_t =
        std::unordered_map<std::string, sdbusplus::message::object_path>;

    /** @brief D-Bus context */
    sdbusplus::async::context& ctx;
    /** @brief Map of pending active events */
    event_map_t pendingEvents;
};

} // namespace cable
