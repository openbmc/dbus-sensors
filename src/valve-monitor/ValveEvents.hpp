#pragma once

#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>

namespace valve
{

class Events
{
  public:
    Events() = delete;

    explicit Events(sdbusplus::async::context& ctx) : ctx(ctx) {}

    auto generateValveEvent(sdbusplus::message::object_path valvePath,
                            bool open) -> sdbusplus::async::task<>;

  private:
    sdbusplus::async::context& ctx;
};

} // namespace valve
