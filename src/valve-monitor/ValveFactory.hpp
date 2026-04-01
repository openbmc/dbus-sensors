#pragma once

#include "BaseValve.hpp"
#include "LocalConfig.hpp"
#include "ValveEvents.hpp"

#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <memory>
#include <string>
#include <vector>

namespace valve
{

class ValveFactory
{
  public:
    /** @brief Get the D-Bus interface names handled by this factory */
    static auto getInterfaces() -> std::vector<std::string>;

    /** @brief Create a valve from the given Entity Manager object path */
    static auto createValve(sdbusplus::async::context& ctx,
                            sdbusplus::message::object_path objectPath,
                            Events& events, const LocalConfig& localConfig,
                            const std::string& interfaceName)
        -> sdbusplus::async::task<std::unique_ptr<BaseValve>>;
};

} // namespace valve
