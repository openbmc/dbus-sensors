#include "dbus-sensor_config.h"

#include "EntityManagerInterface.hpp"
#include "LeakEvents.hpp"
#include "LeakGPIODetector.hpp"

#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Configuration/GPIOLeakDetector/client.hpp>

#include <cstddef>
#include <memory>
#include <optional>
#include <queue>
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
    static constexpr std::size_t maxConcurrentDetectorCreations =
        leakDetectorMaxConcurrentCreations;

    using detector_map_t =
        std::unordered_map<std::string, std::unique_ptr<GPIODetector>>;

    /** @brief  Process new interfaces added to inventory */
    auto processInventoryAdded(const sdbusplus::object_path& objectPath,
                               const std::string& interfaceName) -> void;

    /** @brief Process interfaces removed from inventory */
    auto processInventoryRemoved(const sdbusplus::object_path& objectPath,
                                 const std::string& interfaceName) -> void;

    /** @brief Fill available detector creation slots from the FIFO */
    auto dispatchDetectorCreations() -> void;

    /** @brief Process the config add asynchronously */
    auto processConfigAddedAsync(sdbusplus::object_path objectPath)
        -> sdbusplus::async::task<>;

    /** @brief Get the detector configuration from the Entity Manager */
    auto getDetectorConfig(sdbusplus::object_path objectPath)
        -> sdbusplus::async::task<std::optional<config::DetectorConfig>>;

    sdbusplus::async::context& ctx;
    Events leakEvents;
    entity_manager::EntityManagerInterface entityManager;
    detector_map_t detectors;
    std::queue<sdbusplus::object_path> queuedDetectorCreations;
    std::size_t activeDetectorCreations = 0;
};

} // namespace leak
