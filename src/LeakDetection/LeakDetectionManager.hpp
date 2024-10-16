#include "../EntityManager.hpp"
#include "LeakDetector.hpp"

#include <sdbusplus/async.hpp>

namespace phosphor::leak::manager
{

class LeakDetectionManager;

namespace DetectorIntf = phosphor::leak::detector;
using EMConfigIntf =
    phosphor::entity::manager::EntityManager<LeakDetectionManager>;

class LeakDetectionManager : public EMConfigIntf
{
  public:
    LeakDetectionManager() = delete;

    explicit LeakDetectionManager(sdbusplus::async::context& ctx) :
        EMConfigIntf(ctx), ctx(ctx)
    {
        ctx.spawn(startup());
    }

    /** @brief Process the config added */
    auto processConfigAdded(
        const sdbusplus::message::object_path& objectPath,
        const SensorData& detectorConfig) -> sdbusplus::async::task<>;
    /** @brief Process the config removed */
    auto processConfigRemoved(
        const sdbusplus::message::object_path& objectPath,
        const interface_list_t& interfaces) -> sdbusplus::async::task<>;

  private:
    using detector_map_t =
        std::unordered_map<std::string,
                           std::unique_ptr<DetectorIntf::LeakDetector>>;

    /** @brief Setup and run the Leak Detection Manager */
    auto startup() -> sdbusplus::async::task<>;

    /** @brief D-Bus context */
    sdbusplus::async::context& ctx;
    /** @brief Map of Leak Detectors */
    detector_map_t detectors;
};

} // namespace phosphor::leak::manager
