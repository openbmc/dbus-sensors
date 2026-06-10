/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "NvidiaEventReporting.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"

#include <MessagePackUnpackUtils.hpp>

#include <array>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <span>
#include <system_error>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

using DiscoveryCommands = gpu::DeviceCapabilityDiscoveryCommands;

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

    // The event handler registry is process wide, so each test uses its own
    // EID rather than leaving a registration behind for the next one.
    static std::shared_ptr<NvidiaEventReportingConfig> createConfig(uint8_t eid)
    {
        return std::make_shared<NvidiaEventReportingConfig>(
            eid, requester(),
            std::initializer_list<EventDescriptor>{
                {gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY,
                 static_cast<uint8_t>(
                     gpu::DeviceCapabilityDiscoveryEvents::REDISCOVERY),
                 [](const EventInfo&, std::span<const uint8_t>) {}}});
    }

    static gpu::DeviceCapabilities queriedCaps(
        std::initializer_list<DiscoveryCommands> commands)
    {
        gpu::DeviceCapabilities caps{};
        caps.queried = true;
        caps.commands[gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY] =
            bitsOf(commands);
        return caps;
    }

    std::vector<uint8_t> sentCommands;
};

TEST_F(NvidiaEventReportingTest, SetsUpEventsWhenTheDeviceSupportsThem)
{
    createConfig(0x20)->init(
        queriedCaps({DiscoveryCommands::SET_EVENT_SUBSCRIPTION,
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
    createConfig(0x21)->init(
        queriedCaps({DiscoveryCommands::SET_CURRENT_EVENT_SOURCES}));

    EXPECT_THAT(sentCommands, testing::IsEmpty());
}

TEST_F(NvidiaEventReportingTest, SkipsEventSourcesWhenTheDeviceDoesNotSupportIt)
{
    createConfig(0x22)->init(
        queriedCaps({DiscoveryCommands::SET_EVENT_SUBSCRIPTION}));

    EXPECT_THAT(sentCommands, testing::ElementsAre(static_cast<uint8_t>(
                                  DiscoveryCommands::SET_EVENT_SUBSCRIPTION)));
}

TEST_F(NvidiaEventReportingTest, SetsUpEventsWhenTheDeviceWasNeverQueried)
{
    const gpu::DeviceCapabilities caps{};
    ASSERT_FALSE(caps.queried);

    createConfig(0x23)->init(caps);

    EXPECT_THAT(
        sentCommands,
        testing::ElementsAre(
            static_cast<uint8_t>(DiscoveryCommands::SET_EVENT_SUBSCRIPTION),
            static_cast<uint8_t>(
                DiscoveryCommands::SET_CURRENT_EVENT_SOURCES)));
}

} // namespace
