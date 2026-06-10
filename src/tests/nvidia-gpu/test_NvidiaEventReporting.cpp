/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaEventReporting.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"

#include <array>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <optional>
#include <span>
#include <system_error>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

// Build a DEVICE_CAPABILITY_DISCOVERY response: header + command + cc +
// reasonCode (no payload), matching decodeResponseCommonHeader expectations.
std::vector<uint8_t> buildCapabilityResponse(
    gpu::DeviceCapabilityDiscoveryCommands command,
    ocp::accelerator_management::CompletionCode cc)
{
    std::vector<uint8_t> buf(ocp::accelerator_management::messageHeaderSize + 4,
                             0);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY));
    pack.pack(static_cast<uint8_t>(command));
    pack.pack(static_cast<uint8_t>(cc));
    pack.pack(static_cast<uint16_t>(0)); // reasonCode
    EXPECT_EQ(pack.getError(), 0);
    return buf;
}

std::vector<uint8_t> subscriptionSuccess()
{
    return buildCapabilityResponse(
        gpu::DeviceCapabilityDiscoveryCommands::SET_EVENT_SUBSCRIPTION,
        ocp::accelerator_management::CompletionCode::SUCCESS);
}

std::vector<uint8_t> sourcesSuccess()
{
    return buildCapabilityResponse(
        gpu::DeviceCapabilityDiscoveryCommands::SET_CURRENT_EVENT_SOURCES,
        ocp::accelerator_management::CompletionCode::SUCCESS);
}

// Build a full event message (header + eventData) for NvidiaEventHandler.
std::vector<uint8_t> buildEventBuffer(uint8_t messageType, uint8_t eventId,
                                      uint8_t eventClass, uint16_t eventState,
                                      const std::vector<uint8_t>& eventData)
{
    std::vector<uint8_t> buf(
        ocp::accelerator_management::eventHeaderSize + eventData.size(), 0);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0, messageType);
    pack.pack(static_cast<uint8_t>(0)); // versionByte
    pack.pack(eventId);
    pack.pack(eventClass);
    pack.pack(eventState);
    pack.pack(static_cast<uint8_t>(eventData.size()));
    for (const uint8_t byte : eventData)
    {
        pack.pack(byte);
    }
    EXPECT_EQ(pack.getError(), 0);
    return buf;
}

// Action that records a copy of the request bytes (the reqMsg span is a view
// into caller-owned memory) and completes the call with the owned response.
auto captureAndRespond(std::vector<std::vector<uint8_t>>& history,
                       std::vector<uint8_t> response)
{
    return
        [&history, response = std::move(response)](
            uint8_t /*eid*/, std::span<const uint8_t> reqMsg, auto callback) {
            history.emplace_back(reqMsg.begin(), reqMsg.end());
            callback(std::error_code{}, response);
        };
}

// NvidiaEventHandler keeps registered handlers in a static map for the life of
// the binary, so anything a registered handler writes to has to outlive the
// test that installed it.
std::optional<std::vector<uint8_t>>& dispatchedEventData()
{
    static std::optional<std::vector<uint8_t>> data;
    return data;
}

using DiscoveryCommands = gpu::DeviceCapabilityDiscoveryCommands;

std::array<uint8_t, gpu::supportedListBitfieldSize> bitsOf(
    std::initializer_list<DiscoveryCommands> commands)
{
    std::array<uint8_t, gpu::supportedListBitfieldSize> bits{};
    for (DiscoveryCommands command : commands)
    {
        const auto code = static_cast<uint8_t>(command);
        bits[code / 8U] |= static_cast<uint8_t>(1U << (code % 8U));
    }
    return bits;
}

gpu::DeviceCapabilities queriedCaps(
    std::initializer_list<DiscoveryCommands> commands)
{
    gpu::DeviceCapabilities caps{};
    caps.queried = true;
    caps.commands[gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY] =
        bitsOf(commands);
    return caps;
}

std::vector<uint8_t> buildSuccessResponse(uint8_t command)
{
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY));
    pack.pack(command);
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(0)); // data size
    return buf;
}

uint8_t requestCommand(std::span<const uint8_t> request)
{
    UnpackBuffer buffer(request);
    ocp::accelerator_management::MessageType messageType{};
    uint8_t instanceId = 0;
    uint8_t nvidiaMessageType = 0;
    ocp::accelerator_management::unpackHeader(
        buffer, gpu::nvidiaPciVendorId, messageType, instanceId,
        nvidiaMessageType);
    uint8_t command = 0;
    buffer.unpack(command);
    return command;
}

// The event handler registry is process wide, so each test uses its own
// EID rather than leaving a registration behind for the next one.
class NvidiaEventReportingTest : public MctpMockTestBase
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
                    const uint8_t command = requestCommand(request);
                    sentCommands.push_back(command);
                    callback(std::error_code{}, buildSuccessResponse(command));
                });
    }

    static std::shared_ptr<NvidiaEventReportingConfig> createConfig(
        uint8_t eid, std::initializer_list<EventDescriptor> events)
    {
        return std::make_shared<NvidiaEventReportingConfig>(eid, requester(),
                                                            events);
    }

    std::vector<uint8_t> sentCommands;
};

TEST_F(NvidiaEventReportingTest, InitSubscriptionFailureStops)
{
    // Exactly one request: a failed subscription must not drive event sources.
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {},
            buildCapabilityResponse(
                gpu::DeviceCapabilityDiscoveryCommands::SET_EVENT_SUBSCRIPTION,
                ocp::accelerator_management::CompletionCode::ERROR)));

    auto cfg = createConfig(
        110,
        {EventDescriptor{gpu::MessageType::PLATFORM_ENVIRONMENTAL, 5,
                         [](const EventInfo&, std::span<const uint8_t>) {}}});
    cfg->init(gpu::DeviceCapabilities{});
}

TEST_F(NvidiaEventReportingTest, InitSubscriptionTransportErrorStops)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    auto cfg = createConfig(
        111,
        {EventDescriptor{gpu::MessageType::PLATFORM_ENVIRONMENTAL, 5,
                         [](const EventInfo&, std::span<const uint8_t>) {}}});
    EXPECT_NO_THROW(cfg->init(gpu::DeviceCapabilities{}));
}

TEST_F(NvidiaEventReportingTest, InitDrivesEventSourcesForRegisteredTypes)
{
    // subscription request + one event-sources request for the single
    // non-empty message type.
    std::vector<std::vector<uint8_t>> history;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(captureAndRespond(history, subscriptionSuccess()))
        .WillOnce(captureAndRespond(history, sourcesSuccess()));

    auto cfg = createConfig(
        112,
        {EventDescriptor{gpu::MessageType::PLATFORM_ENVIRONMENTAL, 5,
                         [](const EventInfo&, std::span<const uint8_t>) {}}});
    cfg->init(gpu::DeviceCapabilities{});

    ASSERT_EQ(history.size(), 2U);

    UnpackBuffer sub(history[0]);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  sub, gpu::nvidiaPciVendorId, ocpMsgType, instanceId, msgType),
              0);
    uint8_t subCommand = 0;
    sub.unpack(subCommand);
    EXPECT_EQ(
        subCommand,
        static_cast<uint8_t>(
            gpu::DeviceCapabilityDiscoveryCommands::SET_EVENT_SUBSCRIPTION));

    UnpackBuffer src(history[1]);
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  src, gpu::nvidiaPciVendorId, ocpMsgType, instanceId, msgType),
              0);
    uint8_t srcCommand = 0;
    src.unpack(srcCommand);
    EXPECT_EQ(
        srcCommand,
        static_cast<uint8_t>(
            gpu::DeviceCapabilityDiscoveryCommands::SET_CURRENT_EVENT_SOURCES));
}

TEST_F(NvidiaEventReportingTest, InitVerifiesSubscriptionRequestEncoding)
{
    // Empty response fails decoding, so init stops after the one request.
    std::vector<std::vector<uint8_t>> history;
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(captureAndRespond(history, {}));

    auto cfg = createConfig(
        113,
        {EventDescriptor{gpu::MessageType::PLATFORM_ENVIRONMENTAL, 5,
                         [](const EventInfo&, std::span<const uint8_t>) {}}});
    cfg->init(gpu::DeviceCapabilities{});

    ASSERT_EQ(history.size(), 1U);
    const std::vector<uint8_t>& req = history[0];
    ASSERT_FALSE(req.empty());

    UnpackBuffer unpack(req);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId,
                  msgType),
              0);
    EXPECT_EQ(msgType, static_cast<uint8_t>(
                           gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY));

    uint8_t command = 0;
    uint8_t dataSize = 0;
    uint8_t generationSetting = 0;
    uint8_t targetEid = 0;
    unpack.unpack(command);
    unpack.unpack(dataSize);
    unpack.unpack(generationSetting);
    unpack.unpack(targetEid);
    EXPECT_EQ(
        command,
        static_cast<uint8_t>(
            gpu::DeviceCapabilityDiscoveryCommands::SET_EVENT_SUBSCRIPTION));
    EXPECT_EQ(generationSetting, 2);
    EXPECT_EQ(targetEid, 8); // bmc_eid
    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaEventReportingTest, InitEventSourcesEncodingHasMask)
{
    std::vector<std::vector<uint8_t>> history;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(captureAndRespond(history, subscriptionSuccess()))
        .WillOnce(captureAndRespond(history, sourcesSuccess()));

    constexpr uint8_t eventCode = 5;
    auto cfg = createConfig(
        114,
        {EventDescriptor{gpu::MessageType::PLATFORM_ENVIRONMENTAL, eventCode,
                         [](const EventInfo&, std::span<const uint8_t>) {}}});
    cfg->init(gpu::DeviceCapabilities{});

    ASSERT_EQ(history.size(), 2U);

    UnpackBuffer src(history[1]);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  src, gpu::nvidiaPciVendorId, ocpMsgType, instanceId, msgType),
              0);

    uint8_t command = 0;
    uint8_t dataSize = 0;
    uint8_t messageType = 0;
    uint64_t mask = 0;
    src.unpack(command);
    src.unpack(dataSize);
    src.unpack(messageType);
    src.unpack(mask);
    EXPECT_EQ(dataSize, 9);
    EXPECT_EQ(messageType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    EXPECT_EQ(mask, (1ULL << eventCode));
    EXPECT_EQ(src.getError(), 0);
}

TEST_F(NvidiaEventReportingTest, HandleEventDispatchesToRegisteredHandler)
{
    constexpr uint8_t eid = 100;
    constexpr uint8_t eventId = 0x01;
    dispatchedEventData().reset();

    NvidiaEventHandler::registerEventHandler(
        eid, gpu::MessageType::PLATFORM_ENVIRONMENTAL, eventId,
        [](const EventInfo&, std::span<const uint8_t> eventData) {
            dispatchedEventData() =
                std::vector<uint8_t>(eventData.begin(), eventData.end());
        });

    const std::vector<uint8_t> eventData{0xDE, 0xAD, 0xBE, 0xEF};
    const auto buf = buildEventBuffer(
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL), eventId,
        1, 0, eventData);

    NvidiaEventHandler::handleEvent(eid, buf);

    ASSERT_TRUE(dispatchedEventData().has_value());
    EXPECT_EQ(dispatchedEventData().value_or(std::vector<uint8_t>{}),
              eventData);
}

TEST_F(NvidiaEventReportingTest, HandleEventNoHandlerNoCrash)
{
    const auto buf = buildEventBuffer(
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL), 0x02, 1,
        0, {0x01});
    // eid 101 has no registered handler.
    EXPECT_NO_THROW(NvidiaEventHandler::handleEvent(101, buf));
}

TEST_F(NvidiaEventReportingTest, HandleEventDecodeErrorNoCrash)
{
    const std::vector<uint8_t> buf{0x00, 0x01, 0x02}; // too short
    EXPECT_NO_THROW(NvidiaEventHandler::handleEvent(102, buf));
}

TEST_F(NvidiaEventReportingTest, SetsUpEventsWhenTheDeviceSupportsThem)
{
    createConfig(
        0x20,
        {EventDescriptor{gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY,
                         static_cast<uint8_t>(
                             gpu::DeviceCapabilityDiscoveryEvents::REDISCOVERY),
                         [](const EventInfo&, std::span<const uint8_t>) {}}})
        ->init(queriedCaps({DiscoveryCommands::SET_EVENT_SUBSCRIPTION,
                            DiscoveryCommands::SET_CURRENT_EVENT_SOURCES}));

    EXPECT_THAT(
        sentCommands,
        testing::ElementsAre(
            static_cast<uint8_t>(DiscoveryCommands::SET_EVENT_SUBSCRIPTION),
            static_cast<uint8_t>(
                DiscoveryCommands::SET_CURRENT_EVENT_SOURCES)));
}

TEST_F(NvidiaEventReportingTest, SkipsSubscriptionWhenTheDeviceDoesNotSupportIt)
{
    createConfig(
        0x21,
        {EventDescriptor{gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY,
                         static_cast<uint8_t>(
                             gpu::DeviceCapabilityDiscoveryEvents::REDISCOVERY),
                         [](const EventInfo&, std::span<const uint8_t>) {}}})
        ->init(queriedCaps({DiscoveryCommands::SET_CURRENT_EVENT_SOURCES}));

    EXPECT_THAT(sentCommands, testing::IsEmpty());
}

TEST_F(NvidiaEventReportingTest, SkipsEventSourcesWhenTheDeviceDoesNotSupportIt)
{
    createConfig(
        0x22,
        {EventDescriptor{gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY,
                         static_cast<uint8_t>(
                             gpu::DeviceCapabilityDiscoveryEvents::REDISCOVERY),
                         [](const EventInfo&, std::span<const uint8_t>) {}}})
        ->init(queriedCaps({DiscoveryCommands::SET_EVENT_SUBSCRIPTION}));

    EXPECT_THAT(sentCommands, testing::ElementsAre(static_cast<uint8_t>(
                                  DiscoveryCommands::SET_EVENT_SUBSCRIPTION)));
}

TEST_F(NvidiaEventReportingTest, SetsUpEventsWhenTheDeviceWasNeverQueried)
{
    const gpu::DeviceCapabilities caps{};
    ASSERT_FALSE(caps.queried);

    createConfig(
        0x23,
        {EventDescriptor{gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY,
                         static_cast<uint8_t>(
                             gpu::DeviceCapabilityDiscoveryEvents::REDISCOVERY),
                         [](const EventInfo&, std::span<const uint8_t>) {}}})
        ->init(caps);

    EXPECT_THAT(
        sentCommands,
        testing::ElementsAre(
            static_cast<uint8_t>(DiscoveryCommands::SET_EVENT_SUBSCRIPTION),
            static_cast<uint8_t>(
                DiscoveryCommands::SET_CURRENT_EVENT_SOURCES)));
}

} // namespace
