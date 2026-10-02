#include "BaseValve.hpp"
#include "EntityManagerInterface.hpp"
#include "LocalConfig.hpp"
#include "ValveEvents.hpp"
#include "ValveFactory.hpp"

#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

namespace valve
{

class ValveMonitor
{
  public:
    ValveMonitor() = delete;
    ValveMonitor(const ValveMonitor&) = delete;
    ValveMonitor(ValveMonitor&&) = delete;
    ValveMonitor& operator=(const ValveMonitor&) = delete;
    ValveMonitor& operator=(ValveMonitor&&) = delete;

    explicit ValveMonitor(sdbusplus::async::context& ctx);

  private:
    using valve_map_t =
        std::unordered_map<std::string, std::unique_ptr<BaseValve>>;

    /** @brief  Process new interfaces added to inventory */
    auto processInventoryAdded(
        const sdbusplus::message::object_path& objectPath,
        const std::string& interfaceName) -> void;

    /** @brief Process interfaces removed from inventory */
    auto processInventoryRemoved(
        const sdbusplus::message::object_path& objectPath,
        const std::string& interfaceName) -> void;

    /** @brief Reserve and schedule an AnalogValve creation */
    auto scheduleAnalogCreate(const std::string& objectPath) -> void;

    /** @brief Remove drained AnalogValves and process pending additions */
    auto cleanupRetiringAsync() -> sdbusplus::async::task<>;

    /** @brief Process a GPIO config add using the legacy creation flow */
    auto processGPIOConfigAddedAsync(sdbusplus::message::object_path objectPath,
                                     std::string interfaceName)
        -> sdbusplus::async::task<>;

    /** @brief Process a reserved AnalogValve creation */
    auto processAnalogConfigAddedAsync(
        sdbusplus::message::object_path objectPath, std::string interfaceName,
        std::uint64_t generation) -> sdbusplus::async::task<>;

    struct AnalogState
    {
        std::uint64_t generation = 0;
        bool desiredPresent = false;
        bool creating = false;
        bool retiring = false;
    };

    sdbusplus::async::context& ctx;
    Events events;
    LocalConfig valveConfig;
    entity_manager::EntityManagerInterface entityManager;
    valve_map_t valves;
    std::unordered_map<std::string, AnalogState> analogStates;
};
} // namespace valve
