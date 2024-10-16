#include "LeakDetector.hpp"

#include <sdbusplus/async.hpp>

namespace phosphor::leak::manager
{
namespace DetectorIntf = phosphor::leak::detector;
namespace DetectorConfigIntf = DetectorIntf::config;
namespace RulesIntf = sdbusplus::bus::match::rules;

constexpr auto EntityManagerServiceName = "xyz.openbmc_project.EntityManager";

class LeakDetectionManager
{
  public:
    LeakDetectionManager() = delete;

    explicit LeakDetectionManager(sdbusplus::async::context& ctx) : ctx(ctx)
    {
        // Setup the D-Bus match rules
        configAddedMatch = std::make_unique<sdbusplus::async::match>(
            ctx, RulesIntf::interfacesAdded() +
                     RulesIntf::sender(EntityManagerServiceName));
        configRemovedMatch = std::make_unique<sdbusplus::async::match>(
            ctx, RulesIntf::interfacesRemoved() +
                     RulesIntf::sender(EntityManagerServiceName));
        ctx.spawn(startup());
    }

  private:
    using interface_list_t = std::set<std::string>;
    using detector_map_t =
        std::unordered_map<std::string,
                           std::unique_ptr<DetectorIntf::LeakDetector>>;

    /** @brief Setup and run the Leak Detection Manager */
    auto startup() -> sdbusplus::async::task<>;
    /** @brief Handle async config received from EM */
    auto handleConfigAdded() -> sdbusplus::async::task<>;
    /** @brief Handle async config removed from EM */
    auto handleConfigRemoved() -> sdbusplus::async::task<>;
    /** Get the initial config from EM */
    auto handleConfigGet() -> sdbusplus::async::task<>;
    /** @brief Process the config added */
    auto processConfigAdded(
        const sdbusplus::message::object_path& objectPath,
        const SensorData& detectorConfig) -> sdbusplus::async::task<>;
    /** @brief Process the config removed */
    auto processConfigRemoved(
        const sdbusplus::message::object_path& objectPath,
        const interface_list_t& interfaces) -> sdbusplus::async::task<>;

    /** @brief D-Bus context */
    sdbusplus::async::context& ctx;
    /** @brief Map of Leak Detectors */
    detector_map_t detectors;
    /** @brief sdbusplus::bus::match_t for config added */
    std::unique_ptr<sdbusplus::async::match> configAddedMatch;
    /** @brief sdbusplus::bus::match_t for config removed */
    std::unique_ptr<sdbusplus::async::match> configRemovedMatch;
};

} // namespace phosphor::leak::manager
