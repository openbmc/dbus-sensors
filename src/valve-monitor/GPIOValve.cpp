#include "GPIOValve.hpp"

#include "EntityManagerInterface.hpp"
#include "ValveEvents.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/ObjectMapper/client.hpp>

#include <algorithm>
#include <array>
#include <exception>
#include <functional>
#include <optional>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace valve
{

PHOSPHOR_LOG2_USING;

namespace config
{

/** @brief Polarity name to enum map */
static constexpr std::array<std::pair<std::string_view, PinPolarity>, 2>
    validPinPolarity = {
        {{"Low", PinPolarity::activeLow}, {"High", PinPolarity::activeHigh}}};

auto getConfig(sdbusplus::async::context& ctx,
               sdbusplus::message::object_path objectPath)
    -> sdbusplus::async::task<std::optional<ValveConfig>>
{
    ValveConfig config = {};
    ValveConfigIntf::properties_t properties = {};

    try
    {
        properties =
            co_await ValveConfigIntf(ctx)
                .service(entity_manager::EntityManagerInterface::serviceName)
                .path(objectPath.str)
                .properties();
    }
    catch (const std::exception& e)
    {
        error("Failed to get properties for {PATH}: {ERR}", "PATH",
              objectPath.str, "ERR", e);
        co_return std::nullopt;
    }

    config.name = properties.name;
    config.openPinName = properties.open_pin_name;

    for (const auto& [key, value] : config::validPinPolarity)
    {
        if (properties.open_polarity == key)
        {
            config.openPolarity = value;
            break;
        }
    }
    if (config.openPolarity == config::PinPolarity::unknown)
    {
        error("Invalid polarity {POLARITY} for {NAME}", "POLARITY",
              properties.open_polarity, "NAME", config.name);
        co_return std::nullopt;
    }

    config.openControlPinName = properties.open_control_pin_name;
    config.openControlValue = properties.open_control_value;

    debug(
        "Valve config: {NAME} {OPEN_PIN} {OPEN_POLARITY} {OPEN_CONTROL_PIN} {OPEN_CONTROL_VALUE} ",
        "NAME", config.name, "OPEN_PIN", config.openPinName, "OPEN_POLARITY",
        config.openPolarity, "OPEN_CONTROL_PIN", config.openControlPinName,
        "OPEN_CONTROL_VALUE", config.openControlValue);

    co_return config;
}

} // namespace config

static auto getObjectPath(const std::string& valveName)
{
    return (sdbusplus::message::object_path(ValveIntf::namespace_path::value) /
            ValveIntf::namespace_path::valve_open / valveName);
}

static auto getControlObjectPath(const std::string& valveName)
{
    return (sdbusplus::message::object_path(
                ValveControlIntf::namespace_path::value) /
            ValveControlIntf::namespace_path::valve_open / valveName);
}

constexpr ValveIntf::Value::properties_t initValues{100, 100, 0,
                                                    ValveIntf::Unit::Percent};
constexpr ValveIntf::Definitions::properties_t initAssociations{};
constexpr ValveIntf::Availability::properties_t initAvailability{true};
constexpr ValveIntf::OperationalStatus::properties_t initOperationalState{true};
constexpr ValveControlIntf::Valve::properties_t initControl{true};

GPIOValve::GPIOValve(sdbusplus::async::context& ctx,
                     sdbusplus::message::object_path& objectPath,
                     Events& events, const config::ValveConfig& config) :
    ValveIntf(ctx, getObjectPath(config.name).str.c_str(), initValues,
              initAssociations, initAvailability, initOperationalState),
    ValveControlIntf(ctx, getControlObjectPath(config.name).str.c_str(),
                     initControl),
    ctx(ctx),
    inventoryPath(std::filesystem::path(objectPath.str).parent_path().string()),
    events(events), config(config),
    statusGPIO(ctx, config.name, config.openPinName,
               (config.openPolarity == config::PinPolarity::activeLow),
               std::bind_front(&GPIOValve::updateGPIOStateAsync, this))
{
    ctx.spawn(statusGPIO.start());

    Value::emit_added();
    OperationalStatus::emit_added();
    Availability::emit_added();

    info("Created valve {VALVE}", "VALVE", config.name);
}

GPIOValve::~GPIOValve()
{
    available(false);
}

auto GPIOValve::updateGPIOStateAsync(bool gpioState) -> sdbusplus::async::task<>
{
    auto newValue = gpioState ? 100 : 0;

    if (newValue != value())
    {
        debug("Updating valve {VALVE} to {VALUE}", "VALVE", config.name,
              "VALUE", newValue);
        value(newValue);

        co_await events.generateValveEvent(inventoryPath, gpioState);
    }

    co_return;
}

static auto getContainingChassis(sdbusplus::async::context& ctx,
                                 std::string& objectPath)
    -> sdbusplus::async::task<std::optional<std::string>>
{
    std::vector<std::string> allInterfaces = {
        "xyz.openbmc_project.Inventory.Item.Board",
        "xyz.openbmc_project.Inventory.Item.Chassis",
    };

    auto client = sdbusplus::client::xyz::openbmc_project::ObjectMapper<>(ctx)
                      .service("xyz.openbmc_project.ObjectMapper")
                      .path("/xyz/openbmc_project/object_mapper");
    auto subTree = co_await client.get_sub_tree(
        "/xyz/openbmc_project/inventory/system", 2, allInterfaces);

    // A parent that is a chassis takes precedence
    for (auto& [path, services] : subTree)
    {
        if (path == objectPath)
        {
            co_return path;
        }
    }

    // If no parent is a chassis, return the system chassis
    for (const auto& [path, services] : subTree)
    {
        for (const auto& [service, interfaces] : services)
        {
            if (std::find(interfaces.begin(), interfaces.end(),
                          "xyz.openbmc_project.Inventory.Item.System") !=
                interfaces.end())
            {
                co_return path;
            }
        }
    }

    co_return std::nullopt;
}

auto GPIOValve::createAssociations() -> sdbusplus::async::task<>
{
    using association_t = std::tuple<std::string, std::string, std::string>;
    using association_list_t = std::vector<association_t>;

    association_list_t associationList;
    std::string parent = inventoryPath;

    association_t inventoryAssociation = {"inventory", "sensors", parent};

    associationList.emplace_back(inventoryAssociation);

    auto res = co_await getContainingChassis(ctx, parent);

    association_t chassisAssociation = {"chassis", "all_sensors",
                                        res.value_or(parent)};

    associationList.emplace_back(chassisAssociation);

    associations(associationList);

    Definitions::emit_added();

    co_return;
}

auto GPIOValve::get_property(open_valve_t /*unused*/) const
{
    debug("Getting {VALVE} state", "VALVE", config.name);
    return (value() != 0);
}

auto GPIOValve::set_property(open_valve_t /*unused*/, auto openValve) -> bool
{
    debug("Setting {VALVE} to {VALUE}", "VALVE", config.name, "VALUE",
          openValve ? "open" : "closed");

    try
    {
        gpio::GPIOInterface controlGPIO{ctx, config.name,
                                        config.openControlPinName};
        if (openValve)
        {
            return controlGPIO.setValue(config.openControlValue);
        }
        return (controlGPIO.setValue(!config.openControlValue));
    }
    catch (const std::exception& e)
    {
        error("Failed to set control pin {PIN} to {VALUE}: {ERR}", "PIN",
              config.openControlPinName, "VALUE", config.openControlValue,
              "ERR", e);
    }

    return false;
}

} // namespace valve
