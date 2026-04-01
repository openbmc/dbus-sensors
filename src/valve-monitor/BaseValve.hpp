#pragma once

#include "LocalConfig.hpp"
#include "ValveEvents.hpp"

#include <sdbusplus/async.hpp>
#include <sdbusplus/async/server.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/Control/Valve/aserver.hpp>
#include <xyz/openbmc_project/Sensor/Value/aserver.hpp>
#include <xyz/openbmc_project/State/Decorator/Availability/aserver.hpp>
#include <xyz/openbmc_project/State/Decorator/OperationalStatus/aserver.hpp>

#include <optional>
#include <string>

namespace valve
{

namespace config
{

struct BaseConfig
{
    std::string name = Defaults::name;

    struct Defaults
    {
        static constexpr auto name = "unknown";
    };
};

} // namespace config

class BaseValve;

using ValveIntf = sdbusplus::async::server_t<
    BaseValve, sdbusplus::aserver::xyz::openbmc_project::sensor::Value,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions,
    sdbusplus::aserver::xyz::openbmc_project::state::decorator::Availability,
    sdbusplus::aserver::xyz::openbmc_project::state::decorator::
        OperationalStatus>;

using ValveControlIntf = sdbusplus::async::server_t<
    BaseValve, sdbusplus::aserver::xyz::openbmc_project::control::Valve,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions>;

class BaseValve : public ValveIntf, public ValveControlIntf
{
  public:
    explicit BaseValve(sdbusplus::async::context& ctx,
                       const sdbusplus::message::object_path& objectPath,
                       Events& events, const LocalConfig& localConfig,
                       const config::BaseConfig& config);

    virtual ~BaseValve();

    BaseValve(const BaseValve&) = delete;
    BaseValve(BaseValve&&) = delete;
    BaseValve& operator=(const BaseValve&) = delete;
    BaseValve& operator=(BaseValve&&) = delete;

    /** @brief Emit D-Bus interfaces after construction is complete */
    auto emitInterfaces() -> void;

    /** @brief Create associations for the valve */
    auto createAssociations() -> sdbusplus::async::task<>;

    // NOLINTNEXTLINE(readability-identifier-naming)
    auto get_property(state_t /*unused*/) const -> State;

    // NOLINTNEXTLINE(readability-identifier-naming)
    auto set_property(state_t /*unused*/, auto state) -> bool
    {
        return setState(state);
    }

  protected:
    virtual auto getState() const -> State = 0;
    virtual auto setState(State state) -> bool = 0;

    /** @brief Get the chassis containing the given object path */
    static auto getContainingChassis(sdbusplus::async::context& ctx,
                                     std::string& objectPath)
        -> sdbusplus::async::task<std::optional<std::string>>;

    sdbusplus::async::context& ctx;
    sdbusplus::message::object_path inventoryPath;
    Events& events;
    const LocalConfig& localConfig;
    config::BaseConfig baseConfig;

  private:
    auto createSensorAssociations() -> sdbusplus::async::task<>;

    auto createControlAssociations() -> void;
};

} // namespace valve
