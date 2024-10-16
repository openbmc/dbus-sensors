#include "LeakEvents.hpp"

#include "LeakDetector.hpp"

#include <phosphor-logging/commit.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Logging/Entry/client.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/event.hpp>

#include <string>

PHOSPHOR_LOG2_USING;

namespace leak::events
{

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
sdbusplus::async::task<> LeakEvents::generateLeakEvent(
    sdbusplus::message::object_path detectorPath,
    DetectorIntf::DetectorState state, detector::config::DetectorLevel level)
{
    auto eventName =
        detectorPath.str + "_" + std::to_string(static_cast<int>(level));

    if (state == DetectorIntf::DetectorState::Normal)
    {
        auto pendingEvent = pendingEvents.find(eventName);
        if (pendingEvent != pendingEvents.end())
        {
            co_await resolveLeakEvent(pendingEvent->second);

            using DetectorNormal = sdbusplus::event::xyz::openbmc_project::
                state::leak::Detector::LeakDetectedNormal;
            co_await lg2::commit(ctx,
                                 DetectorNormal("DETECTOR_NAME", detectorPath));

            pendingEvents.erase(eventName);
        }
    }
    else
    {
        namespace ErrorIntf =
            sdbusplus::error::xyz::openbmc_project::state::leak::Detector;
        sdbusplus::message::object_path eventPath{};
        if (level == detector::config::DetectorLevel::critical)
        {
            eventPath = co_await lg2::commit(
                ctx,
                ErrorIntf::LeakDetectedCritical("DETECTOR_NAME", detectorPath));
            error("Critical leak detected for {PATH}", "PATH", detectorPath);
        }
        else
        {
            eventPath = co_await lg2::commit(
                ctx,
                ErrorIntf::LeakDetectedWarning("DETECTOR_NAME", detectorPath));
            warning("Warning leak detected for {PATH}", "PATH", detectorPath);
        }
        pendingEvents[eventName] = eventPath;
    }
    co_return;
}

// NOLINTNEXTLINE(readability-static-accessed-through-instance)
sdbusplus::async::task<> LeakEvents::resolveLeakEvent(
    sdbusplus::message::object_path eventPath)
{
    using LoggingEntry =
        sdbusplus::client::xyz::openbmc_project::logging::Entry<>;

    auto entryClient = LoggingEntry(ctx)
                           .service(LoggingEntry::default_service)
                           .path(eventPath.str);
    co_await entryClient.resolved(true);
}

} // namespace leak::events
