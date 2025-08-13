#pragma once

#include <sdbusplus/async.hpp>
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
    auto generateCableEvent(Type type, std::string name)
        -> sdbusplus::async::task<>;

  private:
    /** @brief Map type for event name to log event object path */
    using event_map_t =
        std::unordered_map<std::string, sdbusplus::message::object_path>;

    sdbusplus::async::context& ctx;
    event_map_t pendingEvents{};
};

} // namespace cable
