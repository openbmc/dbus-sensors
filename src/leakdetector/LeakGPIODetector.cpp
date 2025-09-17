#include "LeakGPIODetector.hpp"

#include "LeakEvents.hpp"
#include "SystemdInterface.hpp"

#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/aserver.hpp>

#include <array>
#include <functional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace leak
{

namespace config
{

/** @brief Leak level to systemd target service map */
static constexpr std::array<
    std::tuple<config::DetectorLevel, std::string_view, std::string_view>, 4>
    leakActionTargets = {
        {{config::DetectorLevel::warning, "assert",
          "xyz.openbmc_project.leakdetector.warning.assert@"},
         {config::DetectorLevel::warning, "deassert",
          "xyz.openbmc_project.leakdetector.warning.deassert@"},
         {config::DetectorLevel::critical, "assert",
          "xyz.openbmc_project.leakdetector.critical.assert@"},
         {config::DetectorLevel::critical, "deassert",
          "xyz.openbmc_project.leakdetector.critical.deassert@"}}};
} // namespace config

static auto getObjectPath(const std::string& detectorName)
    -> sdbusplus::message::object_path
{
    return (
        sdbusplus::message::object_path(DetectorIntf::namespace_path::value) /
        DetectorIntf::namespace_path::detector / detectorName);
}

GPIODetector::GPIODetector(sdbusplus::async::context& ctx, Events& leakEvents,
                           const config::DetectorConfig& config) :
    DetectorIntf(ctx, getObjectPath(config.name).str.c_str(),
                 DetectorIntf::Definitions::properties_t{},
                 DetectorIntf::Detector::properties_t{
                     config.name, DetectorState::Normal, config.type}),
    ctx(ctx), leakEvents(leakEvents), config(config),
    gpioInterface(ctx, config.name, config.pinName,
                  (config.polarity == config::PinPolarity::activeLow),
                  std::bind_front(&GPIODetector::updateGPIOStateAsync, this))
{
    Detector::emit_added();

    createAssociations();

    ctx.spawn(gpioInterface.start());

    debug("Created leak detector {NAME}", "NAME", config.name);
}

auto GPIODetector::createAssociations() -> void
{
    using association_t = std::tuple<std::string, std::string, std::string>;
    using association_list_t = std::vector<association_t>;
    association_list_t associationList;

    association_t association = {"monitoring", "monitored_by",
                                 config.parentInventoryPath};

    associationList.emplace_back(association);

    associations(associationList);

    Definitions::emit_added();
}

auto GPIODetector::updateGPIOStateAsync(bool gpioState)
    -> sdbusplus::async::task<>
{
    auto newState = gpioState ? DetectorIntf::DetectorState::Abnormal
                              : DetectorIntf::DetectorState::Normal;

    debug("Updating detector {DETECTOR} state to {STATE}", "DETECTOR",
          config.name, "STATE", newState);

    if (newState != state_)
    {
        state(newState);

        co_await leakEvents.generateLeakEvent(getObjectPath(config.name),
                                              state_, config.level);
        std::string action = (state_ == DetectorIntf::DetectorState::Normal)
                                 ? "deassert"
                                 : "assert";

        for (const auto& [level, action_str, serviceSuffix] :
             config::leakActionTargets)
        {
            if (config.level == level && action_str == action)
            {
                auto target = std::string(serviceSuffix) + config.name +
                              ".service";
                debug("Starting systemd target {TARGET}", "TARGET", target);
                co_await systemd::SystemdInterface::startUnit(ctx, target);
                break;
            }
        }
    }

    co_return;
}

} // namespace leak
