#include "EntityManager.hpp"
#include "LeakGPIODetector.hpp"

#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Configuration/GPIOLeakDetector/client.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace leak
{

class DetectionManager;

using EMConfigIntf = entity_manager::EntityManager<DetectionManager>;
using GPIODetectorConfigIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::GPIOLeakDetector<>;

class DetectionManager : public EMConfigIntf
{
  public:
    DetectionManager() = delete;

    explicit DetectionManager(sdbusplus::async::context& ctx) :
        EMConfigIntf(ctx, {GPIODetectorConfigIntf::interface}), ctx(ctx),
        leakEvents(ctx)
    {
        ctx.spawn(startup());
    }

    /** @brief  Process new interfaces added to inventory */
    void processInventoryAdded(
        const sdbusplus::message::object_path& objectPath,
        const std::string& interfaceName);

    /** @brief Process interfaces removed from inventory */
    void processInventoryRemoved(
        const sdbusplus::message::object_path& objectPath,
        const std::string& interfaceName);

  private:
    using detector_map_t =
        std::unordered_map<std::string, std::unique_ptr<GPIODetector>>;

    /** @brief Setup and run the Leak Detection Manager */
    sdbusplus::async::task<> startup();

    /** @brief Process the config add asynchronously */
    sdbusplus::async::task<> processConfigAddedAsync(
        sdbusplus::message::object_path objectPath);

    /** @brief Get the detector configuration from the Entity Manager */
    sdbusplus::async::task<config::DetectorConfig> getDetectorConfig(
        sdbusplus::message::object_path objectPath);

    sdbusplus::async::context& ctx;
    detector_map_t detectors;
    Events leakEvents;
};

} // namespace leak
