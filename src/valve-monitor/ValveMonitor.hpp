#include "BaseValve.hpp"
#include "EntityManagerInterface.hpp"
#include "LocalConfig.hpp"
#include "ValveEvents.hpp"
#include "ValveFactory.hpp"

#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>

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

    /** @brief Process the config add asynchronously */
    auto processConfigAddedAsync(sdbusplus::message::object_path objectPath,
                                 std::string interfaceName)
        -> sdbusplus::async::task<>;

    sdbusplus::async::context& ctx;
    Events events;
    LocalConfig valveConfig;
    entity_manager::EntityManagerInterface entityManager;
    valve_map_t valves;
};
} // namespace valve
