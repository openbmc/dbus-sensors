#include "EntityManager.hpp"
#include "LeakDetector.hpp"

#include <sdbusplus/async.hpp>

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
                           std::unique_ptr<detector::LeakDetector>>;

    /** @brief Setup and run the Leak Detection Manager */
    auto startup() -> sdbusplus::async::task<>;

    /** @brief Process the config add asynchronously */
    auto processConfigAddedAsync(sdbusplus::message::object_path objectPath)
        -> sdbusplus::async::task<>;

    /** @brief Get the detector configuration from the Entity Manager */
    auto getDetectorConfig(sdbusplus::message::object_path objectPath)
        -> sdbusplus::async::task<detector::config::DetectorConfig>;

    sdbusplus::async::context& ctx;
    detector_map_t detectors;
    events::LeakEvents leakEvents;
    SystemdIntf systemd;
};

} // namespace phosphor::leak::manager
