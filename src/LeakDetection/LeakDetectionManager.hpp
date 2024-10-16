#include "EntityManager.hpp"
#include "LeakDetector.hpp"

#include <sdbusplus/async.hpp>

namespace DetectorIntf = phosphor::leak::detector;

namespace phosphor::leak::manager
{

class LeakDetectionManager;

using EMConfigIntf =
    phosphor::entity::manager::EntityManager<LeakDetectionManager>;

class LeakDetectionManager : public EMConfigIntf
{
  public:
    LeakDetectionManager() = delete;

    explicit LeakDetectionManager(sdbusplus::async::context& ctx) :
        EMConfigIntf(ctx), ctx(ctx), leakEvents(ctx), systemd(ctx)
    {
        ctx.spawn(startup());
    }

    /** @brief  Process new interfaces added to inventory */
    auto
        processInventoryAdded(const sdbusplus::message::object_path& objectPath,
                              const SensorData& inventoryData) -> void;

    /** @brief Process interfaces removed from inventory */
    auto processInventoryRemoved(
        const sdbusplus::message::object_path& objectPath,
        const interface_list_t& interfaces) -> void;

  private:
    using detector_map_t =
        std::unordered_map<std::string,
                           std::unique_ptr<DetectorIntf::LeakDetector>>;

    /** @brief Setup and run the Leak Detection Manager */
    auto startup() -> sdbusplus::async::task<>;

    /** @brief Process the config add asynchronously */
    auto processConfigAddedAsync(const sdbusplus::message::object_path&
                                     objectPath) -> sdbusplus::async::task<>;

    /** @brief D-Bus context */
    sdbusplus::async::context& ctx;
    /** @brief Map of Leak Detectors */
    detector_map_t detectors;
    /** @brief Leak events instance */
    LeakEventsIntf leakEvents;
    /** @brief Systemd instance */
    SystemdIntf systemd;
};

} // namespace phosphor::leak::manager
