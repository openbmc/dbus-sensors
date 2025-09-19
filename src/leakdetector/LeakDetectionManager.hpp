#include "LeakEvents.hpp"
#include "LeakGPIODetector.hpp"
#include "async_task/EntityManagerInterface.hpp"

#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Configuration/GPIOLeakDetector/client.hpp>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

namespace leak
{

class DetectionManager;

using GPIODetectorConfigIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::GPIOLeakDetector<>;

class DetectionManager
{
  public:
    DetectionManager() = delete;

    explicit DetectionManager(sdbusplus::async::context& ctx);

  private:
    using detector_map_t =
        std::unordered_map<std::string, std::unique_ptr<GPIODetector>>;

    /** @brief  Process new interfaces added to inventory */
    auto processInventoryAdded(
        const sdbusplus::message::object_path& objectPath,
        const std::string& interfaceName) -> void;

    /** @brief Process interfaces removed from inventory */
    auto processInventoryRemoved(
        const sdbusplus::message::object_path& objectPath,
        const std::string& interfaceName) -> void;

    /** @brief Process the config add asynchronously */
    auto processConfigAddedAsync(sdbusplus::message::object_path objectPath)
        -> sdbusplus::async::task<>;

    /** @brief Get the detector configuration from the Entity Manager */
    auto getDetectorConfig(sdbusplus::message::object_path objectPath)
        -> sdbusplus::async::task<std::optional<config::DetectorConfig>>;

    sdbusplus::async::context& ctx;
    Events leakEvents;
    entity_manager::EntityManagerInterface entityManager;
    detector_map_t detectors;
};

} // namespace leak
