#pragma once

#include "LocalConfig.hpp"
#include "ValveEvents.hpp"

#include <sdbusplus/async.hpp>
#include <sdbusplus/async/server.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/Control/Valve/common.hpp>
#include <xyz/openbmc_project/Sensor/Value/aserver.hpp>
#include <xyz/openbmc_project/State/Decorator/Availability/aserver.hpp>
#include <xyz/openbmc_project/State/Decorator/OperationalStatus/aserver.hpp>

#include <memory>
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
class ValveControl;

using ValveIntf = sdbusplus::async::server_t<
    BaseValve, sdbusplus::aserver::xyz::openbmc_project::sensor::Value,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions,
    sdbusplus::aserver::xyz::openbmc_project::state::decorator::Availability,
    sdbusplus::aserver::xyz::openbmc_project::state::decorator::
        OperationalStatus>;

class BaseValve : public ValveIntf
{
  public:
    using Valve = sdbusplus::common::xyz::openbmc_project::control::Valve;
    using State = Valve::State;

    explicit BaseValve(sdbusplus::async::context& ctx,
                       const sdbusplus::message::object_path& objectPath,
                       Events& events, const LocalConfig& localConfig,
                       const config::BaseConfig& config);

    virtual ~BaseValve();

    BaseValve(const BaseValve&) = delete;
    BaseValve(BaseValve&&) = delete;
    BaseValve& operator=(const BaseValve&) = delete;
    BaseValve& operator=(BaseValve&&) = delete;

    /** @brief Emit sensor D-Bus interfaces after construction is complete */
    auto emitSensorInterfaces() -> void;

    /** @brief Create associations for the valve sensor */
    auto createSensorAssociations() -> sdbusplus::async::task<>;

  protected:
    virtual auto getState() const -> State = 0;
    virtual auto setState(State state) -> bool = 0;

    static auto convertStateToString(State state) -> std::string
    {
        return Valve::convertStateToString(state);
    }

    /** @brief Publish the control interface once the valve state is known
     *  @return true if the interface was published by this call
     */
    auto publishControlInterface() -> bool;

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
    friend class ValveControl;

    std::unique_ptr<ValveControl> controlInterface;
};

} // namespace valve
