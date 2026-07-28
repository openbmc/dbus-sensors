/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaEventReporting.hpp"
#include "NvidiaGpuEccMode.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaLongRunningHandler.hpp"
#include "OcpMctpVdm.hpp"
#include "SerialQueue.hpp"
#include "TestUtils.hpp"

// NOLINTNEXTLINE(misc-include-cleaner): sd_bus_error lives here
#include <systemd/sd-bus.h>

#include <boost/system/error_code.hpp>
#include <sdbusplus/exception.hpp>
#include <sdbusplus/message.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr const char* eccModeIface =
    "xyz.openbmc_project.Control.Processor.EccMode";
constexpr const char* eccModePathPrefix =
    "/xyz/openbmc_project/control/processor/";

// Long enough to cover the D-Bus round trips the fixture pumps.
constexpr std::chrono::seconds pumpTimeout{5};

// GetEccMode packs both modes into one flags byte.
constexpr uint8_t eccCurrentEnabledBit = 0x01;
constexpr uint8_t eccPendingEnabledBit = 0x02;

// header(9) + dataSize(2) + flags(1)
std::vector<uint8_t> buildGetEccModeResponse(uint8_t flags)
{
    return test_utils::buildPlatformEnvSuccessResponse(
        gpu::PlatformEnvironmentalCommands::GET_ECC_MODE, flags);
}

// SetEccMode carries no payload; the common header is the whole reply.
std::vector<uint8_t> buildSetEccModeResponse()
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::SET_ECC_MODE,
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::SUCCESS),
        0);
}

// An ACCEPTED reply defers completion to a long-running response event. The
// header's instance id is 0 (buildPlatformEnvErrorResponse), so the deferred
// event must carry the same instance id to route back.
std::vector<uint8_t> buildAcceptedSetEccModeResponse()
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::SET_ECC_MODE,
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::ACCEPTED),
        0);
}

uint8_t commandOf(std::span<const uint8_t> request)
{
    return request[ocp::accelerator_management::messageHeaderSize];
}

// SetEccMode request layout: header + command + dataSize(1) + enable(1).
uint8_t enableFlagOf(std::span<const uint8_t> request)
{
    return request.size() == gpu::setEccModeRequestSize
               ? request.back()
               : 0xFFU;
}

} // namespace

class NvidiaGpuEccModeTest : public MctpMockTestBase
{
  protected:
    void SetUp() override
    {
        MctpMockTestBase::SetUp();
        if (testing::Test::IsSkipped())
        {
            return;
        }
        ON_CALL(mctpMock, sendRecvMsg)
            .WillByDefault(
                [this](uint8_t /*eid*/, std::span<const uint8_t> request,
                       auto callback) {
                    sentRequests.emplace_back(request.begin(), request.end());

                    const auto command =
                        static_cast<gpu::PlatformEnvironmentalCommands>(
                            commandOf(request));

                    // Parking the SetEccMode reply keeps the in-flight guard
                    // set, which is what the concurrent-write test needs.
                    if (command ==
                            gpu::PlatformEnvironmentalCommands::SET_ECC_MODE &&
                        holdSetResponse)
                    {
                        heldSetCallback = std::move(callback);
                        return;
                    }

                    callback(setError, buildReply(command));
                });
    }

    std::vector<uint8_t> buildReply(
        gpu::PlatformEnvironmentalCommands command) const
    {
        switch (command)
        {
            case gpu::PlatformEnvironmentalCommands::GET_ECC_MODE:
                return buildGetEccModeResponse(getFlags);
            case gpu::PlatformEnvironmentalCommands::SET_ECC_MODE:
                return acceptSetResponse ? buildAcceptedSetEccModeResponse()
                                         : buildSetEccModeResponse();
            default:
                return {};
        }
    }

    // Deliver a long-running response event for a previously ACCEPTED command,
    // routed to the handler registered under (PLATFORM_ENVIRONMENTAL, command,
    // instanceId).
    void deliverLongRunningResponse(
        gpu::PlatformEnvironmentalCommands command, uint8_t instanceId,
        ocp::accelerator_management::CompletionCode cc)
    {
        EventInfo eventInfo{};
        eventInfo.eventClass = 128; // longRunningResponseEventClass
        eventInfo.eventState = static_cast<uint16_t>(
            static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL) |
            (static_cast<uint16_t>(static_cast<uint8_t>(command)) << 8));
        const std::vector<uint8_t> eventData{instanceId,
                                             static_cast<uint8_t>(cc), 0, 0};
        longRunningHandler->handler(eventInfo, eventData);
    }

    // shared_ptr ownership matches production: the long running commands
    // outlive the constructor and resolve back into this object.
    std::shared_ptr<NvidiaGpuEccMode> createEccMode(
        const std::string& name, uint8_t eid = test_utils::defaultEid)
    {
        queue = std::make_shared<SerialQueue>(ioContext());
        longRunningHandler =
            std::make_shared<NvidiaLongRunningResponseHandler>(ioContext());
        return std::make_shared<NvidiaGpuEccMode>(
            requester(), objects(), name, eid, queue, longRunningHandler);
    }

    static std::string pathFor(const std::string& name)
    {
        return eccModePathPrefix + name;
    }

    size_t countRequests(gpu::PlatformEnvironmentalCommands command) const
    {
        return static_cast<size_t>(
            std::ranges::count_if(sentRequests, [command](const auto& request) {
                return commandOf(request) == static_cast<uint8_t>(command);
            }));
    }

    std::vector<uint8_t> lastRequest(
        gpu::PlatformEnvironmentalCommands command) const
    {
        for (const auto& request : sentRequests | std::views::reverse)
        {
            if (commandOf(request) == static_cast<uint8_t>(command))
            {
                return request;
            }
        }
        return {};
    }

    struct SetResult
    {
        bool succeeded{false};
        std::string errorName;
    };

    // Writes a property on this connection's own object server. Async because
    // sd-bus rejects self-directed synchronous calls with ELOOP.
    static SetResult setBoolProperty(const std::string& path,
                                     const std::string& property, bool value)
    {
        auto result = std::make_shared<std::optional<SetResult>>();

        bus()->async_method_call(
            [result](const boost::system::error_code& ec,
                     sdbusplus::message_t& reply) {
                SetResult outcome;
                outcome.succeeded = !ec;
                // NOLINTNEXTLINE(misc-include-cleaner): from sd-bus.h
                const sd_bus_error* error = reply.get_error();
                if (error != nullptr && error->name != nullptr)
                {
                    outcome.errorName = error->name;
                }
                *result = outcome;
            },
            bus()->get_unique_name(), path, "org.freedesktop.DBus.Properties",
            "Set", eccModeIface, property, std::variant<bool>(value));

        pumpIoUntil([result] { return result->has_value(); }, pumpTimeout);
        return result->value_or(SetResult{false, "timeout"});
    }

    // Flags the mock GPU reports for GetEccMode.
    uint8_t getFlags{0};
    // Error the mock transport reports; default is a healthy transport.
    std::error_code setError;
    bool holdSetResponse{false};
    bool acceptSetResponse{false};
    std::move_only_function<void(const std::error_code&,
                                 std::span<const uint8_t>)>
        heldSetCallback;

    std::vector<std::vector<uint8_t>> sentRequests;
    std::shared_ptr<SerialQueue> queue;
    std::shared_ptr<NvidiaLongRunningResponseHandler> longRunningHandler;
};

// Construction — the control interface exists with both modes reported off.
TEST_F(NvidiaGpuEccModeTest, ConstructorCreatesEccModeInterface)
{
    const std::string name = "ecc_ctor";
    const auto eccMode = createEccMode(name);

    EXPECT_FALSE(getProperty<bool>(pathFor(name), eccModeIface, "Active"));
    EXPECT_FALSE(getProperty<bool>(pathFor(name), eccModeIface, "Enabled"));
}

// Update — a GetEccMode reply with both bits set publishes both properties.
TEST_F(NvidiaGpuEccModeTest, UpdatePublishesBothModes)
{
    const std::string name = "ecc_update_both";
    getFlags = eccCurrentEnabledBit | eccPendingEnabledBit;
    const auto eccMode = createEccMode(name);

    eccMode->update();

    EXPECT_TRUE(getProperty<bool>(pathFor(name), eccModeIface, "Active"));
    EXPECT_TRUE(getProperty<bool>(pathFor(name), eccModeIface, "Enabled"));
}

// Update — Active and Enabled are decoded from separate bits, so a device
// running ECC with no pending change reports Active only.
TEST_F(NvidiaGpuEccModeTest, UpdatePublishesActiveWithoutPending)
{
    const std::string name = "ecc_update_active";
    getFlags = eccCurrentEnabledBit;
    const auto eccMode = createEccMode(name);

    eccMode->update();

    EXPECT_TRUE(getProperty<bool>(pathFor(name), eccModeIface, "Active"));
    EXPECT_FALSE(getProperty<bool>(pathFor(name), eccModeIface, "Enabled"));
}

// Set — writing Enabled dispatches SetEccMode carrying the requested mode.
TEST_F(NvidiaGpuEccModeTest, SetEnabledTrueSendsSetEccMode)
{
    const std::string name = "ecc_set_true";
    const auto eccMode = createEccMode(name);
    sentRequests.clear();

    const SetResult result = setBoolProperty(pathFor(name), "Enabled", true);
    ASSERT_TRUE(result.succeeded) << result.errorName;

    const std::vector<uint8_t> request =
        lastRequest(gpu::PlatformEnvironmentalCommands::SET_ECC_MODE);
    ASSERT_FALSE(request.empty());
    EXPECT_EQ(enableFlagOf(request), 1U);
}

TEST_F(NvidiaGpuEccModeTest, SetEnabledFalseSendsSetEccMode)
{
    const std::string name = "ecc_set_false";
    getFlags = eccCurrentEnabledBit | eccPendingEnabledBit;
    const auto eccMode = createEccMode(name);
    eccMode->update();
    sentRequests.clear();

    const SetResult result = setBoolProperty(pathFor(name), "Enabled", false);
    ASSERT_TRUE(result.succeeded) << result.errorName;

    const std::vector<uint8_t> request =
        lastRequest(gpu::PlatformEnvironmentalCommands::SET_ECC_MODE);
    ASSERT_FALSE(request.empty());
    EXPECT_EQ(enableFlagOf(request), 0U);
}

// Set — a second write while one is still in flight is refused rather than
// queued, so the caller learns the device is busy.
TEST_F(NvidiaGpuEccModeTest, SetWhileInFlightRejected)
{
    const std::string name = "ecc_set_busy";
    holdSetResponse = true;
    const auto eccMode = createEccMode(name);

    const SetResult first = setBoolProperty(pathFor(name), "Enabled", true);
    ASSERT_TRUE(first.succeeded) << first.errorName;
    ASSERT_TRUE(heldSetCallback);

    const SetResult second = setBoolProperty(pathFor(name), "Enabled", false);
    EXPECT_FALSE(second.succeeded);
    EXPECT_EQ(second.errorName, "xyz.openbmc_project.Common.Error.Unavailable");

    // Release the parked reply so the guard clears before teardown.
    holdSetResponse = false;
    heldSetCallback(std::error_code{}, buildSetEccModeResponse());
    drainPendingAsync();
}

// Set — a failed set must not wedge the in-flight guard.
TEST_F(NvidiaGpuEccModeTest, SetFailureDoesNotBlockLaterSet)
{
    const std::string name = "ecc_set_recover";
    setError = std::make_error_code(std::errc::io_error);
    const auto eccMode = createEccMode(name);

    const SetResult failed = setBoolProperty(pathFor(name), "Enabled", true);
    ASSERT_TRUE(failed.succeeded) << failed.errorName;

    setError = std::error_code{};
    sentRequests.clear();

    const SetResult retried = setBoolProperty(pathFor(name), "Enabled", true);
    EXPECT_TRUE(retried.succeeded) << retried.errorName;
    EXPECT_GT(countRequests(gpu::PlatformEnvironmentalCommands::SET_ECC_MODE),
              0U);
}

// Set — a successful set re-reads the device so Active and Enabled refresh
// without waiting for the next poll.
TEST_F(NvidiaGpuEccModeTest, SetSuccessReadsBackFromDevice)
{
    const std::string name = "ecc_set_readback";
    const auto eccMode = createEccMode(name);
    sentRequests.clear();

    getFlags = eccPendingEnabledBit;
    const SetResult result = setBoolProperty(pathFor(name), "Enabled", true);
    ASSERT_TRUE(result.succeeded) << result.errorName;

    EXPECT_GT(countRequests(gpu::PlatformEnvironmentalCommands::GET_ECC_MODE),
              0U);
    EXPECT_TRUE(getProperty<bool>(pathFor(name), eccModeIface, "Enabled"));
}

// Active mirrors hardware and must not be writable.
TEST_F(NvidiaGpuEccModeTest, ActiveIsReadOnly)
{
    const std::string name = "ecc_active_ro";
    const auto eccMode = createEccMode(name);

    const SetResult result = setBoolProperty(pathFor(name), "Active", true);
    EXPECT_FALSE(result.succeeded);
    // Assert the specific rejection so the test cannot pass on a timeout.
    EXPECT_EQ(result.errorName, "org.freedesktop.DBus.Error.PropertyReadOnly");
}

// Set — an ACCEPTED reply defers completion to the long-running response
// event, which then finishes the set and re-reads the device.
TEST_F(NvidiaGpuEccModeTest, SetAcceptedCompletesViaLongRunningResponse)
{
    const std::string name = "ecc_set_accepted";
    const auto eccMode = createEccMode(name);

    acceptSetResponse = true;
    getFlags = eccPendingEnabledBit;

    const SetResult accepted = setBoolProperty(pathFor(name), "Enabled", true);
    ASSERT_TRUE(accepted.succeeded) << accepted.errorName;

    // Only ACCEPTED so far: the deferred response has not arrived, so the
    // in-flight guard is still held and a concurrent set is rejected.
    const SetResult concurrent =
        setBoolProperty(pathFor(name), "Enabled", false);
    EXPECT_FALSE(concurrent.succeeded);
    EXPECT_EQ(concurrent.errorName,
              "xyz.openbmc_project.Common.Error.Unavailable");

    sentRequests.clear();

    // Deliver the deferred SUCCESS for the parked set.
    deliverLongRunningResponse(
        gpu::PlatformEnvironmentalCommands::SET_ECC_MODE, 0,
        ocp::accelerator_management::CompletionCode::SUCCESS);
    drainPendingAsync();

    // The long-running success path finished the set and re-read the device.
    EXPECT_GT(countRequests(gpu::PlatformEnvironmentalCommands::GET_ECC_MODE),
              0U);
    EXPECT_TRUE(getProperty<bool>(pathFor(name), eccModeIface, "Enabled"));
}

// A device that rejects the read leaves the published values untouched.
TEST_F(NvidiaGpuEccModeTest, UpdateBadCompletionCodeKeepsPreviousValues)
{
    const std::string name = "ecc_bad_cc";
    const auto eccMode = createEccMode(name);

    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillRepeatedly(mock_mctp::respondWith(
            std::error_code{},
            test_utils::buildPlatformEnvErrorResponse(
                gpu::PlatformEnvironmentalCommands::GET_ECC_MODE,
                static_cast<uint8_t>(
                    ocp::accelerator_management::CompletionCode::
                        ERR_UNSUPPORTED_COMMAND_CODE),
                0)));

    eccMode->update();

    EXPECT_FALSE(getProperty<bool>(pathFor(name), eccModeIface, "Active"));
    EXPECT_FALSE(getProperty<bool>(pathFor(name), eccModeIface, "Enabled"));
}

// A transport failure must not take the daemon down.
TEST_F(NvidiaGpuEccModeTest, UpdateMctpTransportErrorNoCrash)
{
    const std::string name = "ecc_transport_error";
    const auto eccMode = createEccMode(name);

    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillRepeatedly(mock_mctp::respondWith(
            std::make_error_code(std::errc::io_error), {}));

    eccMode->update();

    EXPECT_FALSE(getProperty<bool>(pathFor(name), eccModeIface, "Active"));
}

TEST_F(NvidiaGpuEccModeTest, DestructorRemovesDbusInterface)
{
    const std::string name = "ecc_dtor";
    auto eccMode = createEccMode(name);
    eccMode->update();

    // The interface is present while the object is alive.
    EXPECT_NO_THROW(getProperty<bool>(pathFor(name), eccModeIface, "Enabled"));

    eccMode.reset();
    drainPendingAsync();

    // Destruction removes the interface, so reads now throw.
    EXPECT_THROW(getProperty<bool>(pathFor(name), eccModeIface, "Enabled"),
                 sdbusplus::exception_t);
}
