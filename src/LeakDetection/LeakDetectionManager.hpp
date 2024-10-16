#include "LeakDetector.hpp"

#include <sdbusplus/async.hpp>

namespace phosphor::leak::manager
{
namespace LeakDetectorIntf = phosphor::leak::detector;
namespace LeakDetectorConfigIntf = LeakDetectorIntf::config;

class LeakDetectionManager
{
  public:
    LeakDetectionManager() = delete;

    explicit LeakDetectionManager(sdbusplus::async::context& ctx) : ctx(ctx)
    {
        ctx.spawn(startup());
    }

  private:
    /** @brief Setup and run a new health monitor object */
    auto startup() -> sdbusplus::async::task<>;
    /** @brief Run the health monitor */
    auto run() -> sdbusplus::async::task<>;

    // void handleConfigAdded(sdbusplus::message_t& msg);

    auto handleConfigAdded() -> sdbusplus::async::task<>;
    auto handleConfigRemoved() -> sdbusplus::async::task<>;
    auto handleConfigGet() -> sdbusplus::async::task<>;
    auto processConfig(
        const sdbusplus::message::object_path& objectPath,
        const SensorData& detectorConfig) -> sdbusplus::async::task<>;

    using detector_map_t =
        std::unordered_map<std::string,
                           std::unique_ptr<LeakDetectorIntf::LeakDetector>>;

    /** @brief D-Bus context */
    sdbusplus::async::context& ctx;
    /** @brief Map of Leak Detectors */
    detector_map_t detectors;
    /** @brief sdbusplus::bus::match_t for config added */
    std::unique_ptr<sdbusplus::bus::match_t> configAddedMatch;
    /** @brief sdbusplus::bus::match_t for config removed */
    std::unique_ptr<sdbusplus::bus::match_t> configRemovedMatch;
};

} // namespace phosphor::leak::manager
