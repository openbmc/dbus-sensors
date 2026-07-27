#include "BaseValve.hpp"

#include "LocalConfig.hpp"
#include "ValveEvents.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/Control/Valve/aserver.hpp>
#include <xyz/openbmc_project/ObjectMapper/client.hpp>
#include <xyz/openbmc_project/Sensor/Value/aserver.hpp>
#include <xyz/openbmc_project/State/Decorator/Availability/aserver.hpp>
#include <xyz/openbmc_project/State/Decorator/OperationalStatus/aserver.hpp>

#include <algorithm>
#include <filesystem>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace valve
{

PHOSPHOR_LOG2_USING;

using association_t = std::tuple<std::string, std::string, std::string>;
using association_list_t = std::vector<association_t>;

using ValveControlIntf = sdbusplus::async::server_t<
    ValveControl, sdbusplus::aserver::xyz::openbmc_project::control::Valve,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions>;

class ValveControl : public ValveControlIntf
{
  public:
    ValveControl(sdbusplus::async::context& ctx,
                 const sdbusplus::object_path& objectPath, BaseValve& valve) :
        ValveControlIntf(ctx, objectPath), valve(valve)
    {}

    auto emitInterfaces(const sdbusplus::object_path& inventoryPath) -> void
    {
        associations({{"controlling", "controlled_by", inventoryPath}});
        Definitions::emit_added();
        Valve::emit_added();
    }

    // NOLINTNEXTLINE(readability-identifier-naming)
    auto get_property(state_t /*unused*/) const -> State
    {
        return valve.getState();
    }

    // NOLINTNEXTLINE(readability-identifier-naming)
    auto set_property(state_t /*unused*/, State state) -> bool
    {
        return valve.setState(state);
    }

  private:
    BaseValve& valve;
};

static auto getObjectPath(std::string valveName)
{
    std::replace(valveName.begin(), valveName.end(), ' ', '_');

    return (sdbusplus::message::object_path(ValveIntf::namespace_path::value) /
            ValveIntf::namespace_path::valve / valveName);
}

static auto getControlObjectPath(std::string valveName)
{
    std::replace(valveName.begin(), valveName.end(), ' ', '_');

    return (sdbusplus::message::object_path(ValveControlIntf::namespace_path) /
            valveName);
}

constexpr ValveIntf::Value::properties_t initValues{0, 100, 0,
                                                    ValveIntf::Unit::Percent};
constexpr ValveIntf::Definitions::properties_t initAssociations{};
constexpr ValveIntf::Availability::properties_t initAvailability{true};
constexpr ValveIntf::OperationalStatus::properties_t initOperationalState{true};

BaseValve::BaseValve(sdbusplus::async::context& ctx,
                     const sdbusplus::message::object_path& objectPath,
                     Events& events, const LocalConfig& localConfig,
                     const config::BaseConfig& config) :
    ValveIntf(ctx, getObjectPath(config.name).str.c_str(), initValues,
              initAssociations, initAvailability, initOperationalState),
    ctx(ctx),
    inventoryPath(std::filesystem::path(objectPath.str).parent_path().string()),
    events(events), localConfig(localConfig), baseConfig(config)
{
    info("Created valve {VALVE}", "VALVE", baseConfig.name);
}

auto BaseValve::emitSensorInterfaces() -> void
{
    Value::emit_added();
    OperationalStatus::emit_added();
    Availability::emit_added();
}

BaseValve::~BaseValve()
{
    available(false);
}

auto BaseValve::publishControlInterface() -> bool
{
    if (controlInterface)
    {
        return false;
    }

    controlInterface = std::make_unique<ValveControl>(
        ctx, getControlObjectPath(baseConfig.name), *this);
    controlInterface->emitInterfaces(inventoryPath);
    return true;
}

auto BaseValve::createSensorAssociations() -> sdbusplus::async::task<>
{
    association_list_t associationList;
    std::string parent = inventoryPath;

    association_t inventoryAssociation = {"inventory", "sensors",
                                          inventoryPath};

    associationList.emplace_back(inventoryAssociation);

    auto res = co_await getContainingChassis(ctx, parent);

    association_t chassisAssociation = {"chassis", "all_sensors",
                                        res.value_or(parent)};

    associationList.emplace_back(chassisAssociation);

    ValveIntf::associations(associationList);

    ValveIntf::Definitions::emit_added();

    co_return;
}

auto BaseValve::getContainingChassis(sdbusplus::async::context& ctx,
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

} // namespace valve
