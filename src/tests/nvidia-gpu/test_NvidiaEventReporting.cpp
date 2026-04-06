/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaEventReporting.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"

#include <MessagePackUnpackUtils.hpp>

#include <cstdint>
#include <initializer_list>
#include <memory>
#include <span>
#include <system_error>
#include <vector>

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

class NvidiaEventReportingTestBase : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaEventReportingConfig> createConfig(
        uint8_t eid, std::initializer_list<EventDescriptor> events)
    {
        return std::make_shared<NvidiaEventReportingConfig>(eid, *mctpRequester,
                                                            events);
    }
};

TEST_F(NvidiaEventReportingTestBase, InitSubscriptionFailureStops)
{
    mock_mctp::pushResponse(
        {}, buildCapabilityResponse(
                gpu::DeviceCapabilityDiscoveryCommands::SET_EVENT_SUBSCRIPTION,
                ocp::accelerator_management::CompletionCode::ERROR));
    auto cfg = createConfig(
        110,
        {EventDescriptor{gpu::MessageType::PLATFORM_ENVIRONMENTAL, 5,
                         [](const EventInfo&, std::span<const uint8_t>) {}}});
    cfg->init();
    EXPECT_EQ(mock_mctp::getRequestCount(), 1U);
}

TEST_F(NvidiaEventReportingTestBase, InitSubscriptionTransportErrorStops)
{
    mock_mctp::pushResponse(std::make_error_code(std::errc::timed_out), {});
    auto cfg = createConfig(
        111,
        {EventDescriptor{gpu::MessageType::PLATFORM_ENVIRONMENTAL, 5,
                         [](const EventInfo&, std::span<const uint8_t>) {}}});
    EXPECT_NO_THROW(cfg->init());
    EXPECT_EQ(mock_mctp::getRequestCount(), 1U);
}

TEST_F(NvidiaEventReportingTestBase, InitDrivesEventSourcesForRegisteredTypes)
{
    mock_mctp::pushResponse({}, subscriptionSuccess());
    mock_mctp::pushResponse({}, sourcesSuccess());
    auto cfg = createConfig(
        112,
        {EventDescriptor{gpu::MessageType::PLATFORM_ENVIRONMENTAL, 5,
                         [](const EventInfo&, std::span<const uint8_t>) {}}});
    cfg->init();

    // subscription request + one event-sources request for the single
    // non-empty message type.
    EXPECT_EQ(mock_mctp::getRequestCount(), 2U);

    const auto& history = mock_mctp::getRequestHistory();
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

TEST_F(NvidiaEventReportingTestBase, InitVerifiesSubscriptionRequestEncoding)
{
    mock_mctp::setNextResponse({}, {});
    auto cfg = createConfig(
        113,
        {EventDescriptor{gpu::MessageType::PLATFORM_ENVIRONMENTAL, 5,
                         [](const EventInfo&, std::span<const uint8_t>) {}}});
    cfg->init();

    const std::span<const uint8_t> req = mock_mctp::getLastRequest();
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

TEST_F(NvidiaEventReportingTestBase, InitEventSourcesEncodingHasMask)
{
    mock_mctp::pushResponse({}, subscriptionSuccess());
    mock_mctp::pushResponse({}, sourcesSuccess());
    constexpr uint8_t eventCode = 5;
    auto cfg = createConfig(
        114,
        {EventDescriptor{gpu::MessageType::PLATFORM_ENVIRONMENTAL, eventCode,
                         [](const EventInfo&, std::span<const uint8_t>) {}}});
    cfg->init();

    const auto& history = mock_mctp::getRequestHistory();
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

TEST_F(NvidiaEventReportingTestBase, HandleEventDispatchesToRegisteredHandler)
{
    constexpr uint8_t eid = 100;
    constexpr uint8_t eventId = 0x01;
    bool called = false;
    std::vector<uint8_t> capturedData;

    NvidiaEventHandler::registerEventHandler(
        eid, gpu::MessageType::PLATFORM_ENVIRONMENTAL, eventId,
        [&](const EventInfo&, std::span<const uint8_t> eventData) {
            called = true;
            capturedData.assign(eventData.begin(), eventData.end());
        });

    const std::vector<uint8_t> eventData{0xDE, 0xAD, 0xBE, 0xEF};
    const auto buf = buildEventBuffer(
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL), eventId,
        1, 0, eventData);

    NvidiaEventHandler::handleEvent(eid, buf);

    EXPECT_TRUE(called);
    EXPECT_EQ(capturedData, eventData);
}

TEST_F(NvidiaEventReportingTestBase, HandleEventNoHandlerNoCrash)
{
    const auto buf = buildEventBuffer(
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL), 0x02, 1,
        0, {0x01});
    // eid 101 has no registered handler.
    EXPECT_NO_THROW(NvidiaEventHandler::handleEvent(101, buf));
}

TEST_F(NvidiaEventReportingTestBase, HandleEventDecodeErrorNoCrash)
{
    const std::vector<uint8_t> buf{0x00, 0x01, 0x02}; // too short
    EXPECT_NO_THROW(NvidiaEventHandler::handleEvent(102, buf));
}

} // namespace
