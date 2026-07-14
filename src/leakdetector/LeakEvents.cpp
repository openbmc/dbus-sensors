#include "LeakEvents.hpp"

#include "LeakGPIODetector.hpp"

#include <phosphor-logging/commit.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/exception.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/event.hpp>

#include <tuple>

PHOSPHOR_LOG2_USING;

namespace leak
{

auto Events::generateLeakEvent(sdbusplus::object_path detectorPath,
                               DetectorStateIntf::DetectorState state,
                               config::DetectorLevel level)
    -> sdbusplus::async::task<>
{
    auto eventName = std::make_tuple(detectorPath.str, level);

    // NOLINTNEXTLINE(clang-analyzer-core.uninitialized.Branch)
    if (state == DetectorStateIntf::DetectorState::Normal)
    {
        auto pendingEvent = pendingEvents.find(eventName);
        if (pendingEvent != pendingEvents.end())
        {
            try
            {
                co_await lg2::resolve(ctx, pendingEvent->second);
            }
            // Catch internal_exception rather than SdBusError so the handling
            // still covers the finer-grained sd-bus event types sdbusplus is
            // moving to. See
            // https://gerrit.openbmc.org/c/openbmc/dbus-sensors/+/89627
            catch (const sdbusplus::exception::internal_exception& e)
            {
                error("Failed to resolve leak event for {PATH}: {ERROR}",
                      "PATH", detectorPath, "ERROR", e);
            }

            using DetectorNormal = sdbusplus::event::xyz::openbmc_project::
                state::leak::Detector::LeakDetectedNormal;
            try
            {
                co_await lg2::commit(
                    ctx, DetectorNormal("DETECTOR_NAME", detectorPath));
            }
            catch (const sdbusplus::exception::internal_exception& e)
            {
                error("Failed to commit normal leak event for {PATH}: {ERROR}",
                      "PATH", detectorPath, "ERROR", e);
            }

            pendingEvents.erase(eventName);
        }
        co_return;
    }

    namespace error_intf =
        sdbusplus::error::xyz::openbmc_project::state::leak::Detector;
    sdbusplus::object_path eventPath{};

    try
    {
        if (level == config::DetectorLevel::critical)
        {
            eventPath =
                co_await lg2::commit(ctx, error_intf::LeakDetectedCritical(
                                              "DETECTOR_NAME", detectorPath));
            error("Critical leak detected for {PATH}", "PATH", detectorPath);
        }
        else
        {
            eventPath = co_await lg2::commit(
                ctx,
                error_intf::LeakDetectedWarning("DETECTOR_NAME", detectorPath));
            warning("Warning leak detected for {PATH}", "PATH", detectorPath);
        }
    }
    catch (const sdbusplus::exception::internal_exception& e)
    {
        error("Failed to commit leak event for {PATH}: {ERROR}", "PATH",
              detectorPath, "ERROR", e);
        co_return;
    }

    pendingEvents[eventName] = eventPath;
}

} // namespace leak
