#include "LeakEvents.hpp"

#include "LeakGPIODetector.hpp"

#include <phosphor-logging/commit.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/event.hpp>

#include <format>

PHOSPHOR_LOG2_USING;

namespace leak
{

sdbusplus::async::task<> Events::generateLeakEvent(
    sdbusplus::message::object_path detectorPath,
    DetectorStateIntf::DetectorState state, config::DetectorLevel level)
{
    auto eventName =
        std::format("{}_{}", detectorPath.str, static_cast<int>(level));

    if (state == DetectorStateIntf::DetectorState::Normal)
    {
        auto pendingEvent = pendingEvents.find(eventName);
        if (pendingEvent != pendingEvents.end())
        {
            co_await lg2::resolve(ctx, pendingEvent->second);

            using DetectorNormal = sdbusplus::event::xyz::openbmc_project::
                state::leak::Detector::LeakDetectedNormal;
            co_await lg2::commit(ctx,
                                 DetectorNormal("DETECTOR_NAME", detectorPath));

            pendingEvents.erase(eventName);
        }
        co_return;
    }

    namespace error_intf =
        sdbusplus::error::xyz::openbmc_project::state::leak::Detector;
    sdbusplus::message::object_path eventPath{};

    if (level == config::DetectorLevel::critical)
    {
        eventPath = co_await lg2::commit(
            ctx,
            error_intf::LeakDetectedCritical("DETECTOR_NAME", detectorPath));
        error("Critical leak detected for {PATH}", "PATH", detectorPath);
    }
    else
    {
        eventPath = co_await lg2::commit(
            ctx,
            error_intf::LeakDetectedWarning("DETECTOR_NAME", detectorPath));
        warning("Warning leak detected for {PATH}", "PATH", detectorPath);
    }
    pendingEvents[eventName] = eventPath;
}

} // namespace leak
