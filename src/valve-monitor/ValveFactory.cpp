#include "ValveFactory.hpp"

#include "AnalogValve.hpp"
#include "GPIOValve.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <memory>
#include <string>
#include <vector>

namespace valve
{

PHOSPHOR_LOG2_USING;

auto ValveFactory::getInterfaces() -> std::vector<std::string>
{
    return {GPIOConfigIntf::interface, AnalogConfigIntf::interface};
}

auto ValveFactory::createValve(sdbusplus::async::context& ctx,
                               sdbusplus::message::object_path objectPath,
                               Events& events, const LocalConfig& localConfig,
                               const std::string& interfaceName)
    -> sdbusplus::async::task<std::unique_ptr<BaseValve>>
{
    std::unique_ptr<BaseValve> valve = nullptr;

    if (interfaceName == GPIOConfigIntf::interface)
    {
        auto res = co_await config::getConfig(ctx, objectPath);
        if (!res.has_value())
        {
            error("Failed to get config for {OBJECT_PATH}", "OBJECT_PATH",
                  objectPath);
            co_return nullptr;
        }

        valve = std::make_unique<GPIOValve>(ctx, objectPath, events,
                                            localConfig, res.value());
    }
    else if (interfaceName == AnalogConfigIntf::interface)
    {
        auto res = co_await config::getAnalogConfig(ctx, objectPath);
        if (!res.has_value())
        {
            error("Failed to get analog config for {OBJECT_PATH}",
                  "OBJECT_PATH", objectPath);
            co_return nullptr;
        }

        valve = std::make_unique<AnalogValve>(ctx, objectPath, events,
                                              localConfig, res.value());
    }
    else
    {
        error("Unknown valve interface {INTERFACE}", "INTERFACE",
              interfaceName);
        co_return nullptr;
    }

    valve->emitInterfaces();
    co_await valve->createAssociations();
    co_return valve;
}

} // namespace valve
