#include "LeakEvents.hpp"

#include <phosphor-logging/commit.hpp>
#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Logging/Entry/client.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/event.hpp>

PHOSPHOR_LOG2_USING;

namespace phosphor::leak::events
{
namespace EventIntf =
    sdbusplus::event::xyz::openbmc_project::state::leak::Detector;

auto LeakEvents::resolveLeakEvent(sdbusplus::message::object_path& eventPath)
    -> sdbusplus::async::task<>
{
    using LoggingEntry =
        sdbusplus::client::xyz::openbmc_project::logging::Entry<>;

    auto entryClient = LoggingEntry(ctx)
                           .service(LoggingEntry::default_service)
                           .path(eventPath.str);
    co_await entryClient.resolved(true);
}

auto LeakEvents::generateLeakEvent(
    const sdbusplus::message::object_path& detectorPath,
    DetectorIntf::DetectorState state,
    DetectorConfigIntf::DetectorLevel level) -> sdbusplus::async::task<>
{
    auto eventName = detectorPath.str + "_" +
                     DetectorConfigIntf::convertDetectorLevelToString(level);

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
        if (level == DetectorConfigIntf::DetectorLevel::Critical)
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
        pendingEvents.emplace(eventName, eventPath);
    }
    co_return;
}
} // namespace phosphor::leak::events
