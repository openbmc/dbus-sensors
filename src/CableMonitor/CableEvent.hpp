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
    auto generateCableEvent(Type type, std::string& name) -> void;

  private:
    /** @brief D-Bus context */
    sdbusplus::async::context& ctx;
};

} // namespace phosphor::cable::event
