/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuRist.hpp"

#include "NvidiaEventReporting.hpp"
#include "NvidiaGpuMctpVdm.hpp"

#include <boost/system/error_code.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <format>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>

namespace
{

constexpr uint8_t ristResultFail = 0;
constexpr uint8_t ristResultPass = 1;
constexpr uint8_t ristStatusClassShift = 62;
constexpr uint64_t ristStatusClassMask = 0x3;
constexpr uint8_t ristStatusClassOk = 0;
constexpr uint8_t ristStatusClassWarning = 1;
constexpr uint8_t ristStatusClassCritical = 2;
constexpr std::string_view ristResolution =
    "Check RIST app output, please contact with NVIDIA for the app and further troubleshooting.";

template <size_t size>
std::string boundedString(const std::array<char, size>& value)
{
    const auto end = std::find(value.begin(), value.end(), '\0');
    return {value.begin(), end};
}

std::string formatTimestamp(uint64_t nanoseconds)
{
    const std::chrono::time_point<std::chrono::system_clock,
                                  std::chrono::nanoseconds>
        timePoint{std::chrono::nanoseconds(nanoseconds)};
    const std::time_t timestamp =
        std::chrono::system_clock::to_time_t(timePoint);
    std::tm localTime{};
    if (localtime_r(&timestamp, &localTime) == nullptr)
    {
        lg2::warning(
            "Failed to convert Runtime IST event timestamp {TIMESTAMP}",
            "TIMESTAMP", nanoseconds);
        return {};
    }

    std::array<char, 100> formatted{};
    if (std::strftime(formatted.data(), formatted.size(), "%c", &localTime) ==
        0)
    {
        lg2::warning("Failed to format Runtime IST event timestamp {TIMESTAMP}",
                     "TIMESTAMP", nanoseconds);
        return {};
    }

    return formatted.data();
}

std::string formatResult(uint8_t result)
{
    if (result == ristResultPass)
    {
        return "Pass";
    }
    if (result == ristResultFail)
    {
        return "Fail";
    }

    lg2::warning("Runtime IST event has unknown result byte {RESULT}", "RESULT",
                 result);
    return std::format("Unknown(0x{:02X})", result);
}

std::pair<std::string_view, std::string_view> deriveSeverityAndResolution(
    uint8_t result, uint64_t statusCode)
{
    const uint8_t statusClass = static_cast<uint8_t>(
        (statusCode >> ristStatusClassShift) & ristStatusClassMask);

    if (result == ristResultPass && statusClass == ristStatusClassOk)
    {
        return {"xyz.openbmc_project.Logging.Entry.Level.Informational",
                "None"};
    }
    if (result == ristResultFail && statusClass == ristStatusClassWarning)
    {
        return {"xyz.openbmc_project.Logging.Entry.Level.Warning",
                ristResolution};
    }
    if (result == ristResultFail && statusClass == ristStatusClassCritical)
    {
        return {"xyz.openbmc_project.Logging.Entry.Level.Critical",
                ristResolution};
    }

    lg2::warning(
        "Runtime IST event has undefined result and status class combination: result={RESULT}, class={CLASS}",
        "RESULT", result, "CLASS", statusClass);
    return {"xyz.openbmc_project.Logging.Entry.Level.Critical", ristResolution};
}

} // namespace

NvidiaRistEventHandler::NvidiaRistEventHandler(
    const std::string& deviceName,
    const std::shared_ptr<sdbusplus::asio::connection>& conn) :
    deviceName(deviceName), conn(conn)
{}

void NvidiaRistEventHandler::handleLoggingComplete(
    const boost::system::error_code& ec)
{
    if (ec)
    {
        lg2::error("Failed to log RIST event for device {DEV}: {EC}", "DEV",
                   deviceName, "EC", ec.message());
    }
}

void NvidiaRistEventHandler::handleRistEvent(const EventInfo& /* eventInfo */,
                                             std::span<const uint8_t> eventData)
{
    gpu::RistEventData ristEvent{};
    const int rc = gpu::decodeRistEvent(eventData, ristEvent);
    if (rc != 0)
    {
        lg2::error("Failed to decode RIST event for device {DEV}", "DEV",
                   deviceName);
        return;
    }

    const std::string& resourceName = deviceName;
    const std::string result = formatResult(ristEvent.result);
    const std::string timestamp = formatTimestamp(ristEvent.timestamp);
    const double maxTemperature =
        static_cast<double>(ristEvent.maxTemperature) / 256.0;
    const double avgTemperature =
        static_cast<double>(ristEvent.avgTemperature) / 256.0;

    const std::string eventDetails = std::format(
        "Runtime IST event [Result={}][AppVersion={}][StatusCode=0x{:016X}]"
        "[MaxTemperature={:.2f}C][AvgTemperature={:.2f}C][Timestamp={}]"
        "[UUID={}]",
        result, boundedString(ristEvent.appVersion), ristEvent.statusCode,
        maxTemperature, avgTemperature, timestamp,
        boundedString(ristEvent.gpuIdentifier));
    const std::string eventMessage = std::format(
        "The resource property {} has detected errors of type '{}'.",
        resourceName, eventDetails);

    const auto [severity, resolution] =
        deriveSeverityAndResolution(ristEvent.result, ristEvent.statusCode);
    const std::map<std::string, std::string> additionalData{
        {"REDFISH_ORIGIN_OF_CONDITION",
         std::format("/xyz/openbmc_project/inventory/{}", deviceName)},
        {"REDFISH_MESSAGE_ARGS",
         std::format("{},{}", resourceName, eventDetails)},
        {"REDFISH_MESSAGE_ID", "ResourceEvent.1.0.ResourceErrorsDetected"},
        {"namespace", resourceName},
        {"xyz.openbmc_project.Logging.Entry.EventId", "GPU-RIST-EVENT"},
        {"xyz.openbmc_project.Logging.Entry.Resolution",
         std::string(resolution)}};

    conn->async_method_call(
        [weak{weak_from_this()}](const boost::system::error_code& ec) {
            std::shared_ptr<NvidiaRistEventHandler> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaRistEventHandler");
                return;
            }
            self->handleLoggingComplete(ec);
        },
        "xyz.openbmc_project.Logging", "/xyz/openbmc_project/logging",
        "xyz.openbmc_project.Logging.Create", "Create", eventMessage,
        std::string(severity), additionalData);
}
