/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaDeviceSupportedCommandCodes.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <MessagePackUnpackUtils.hpp>

#include <array>
#include <cstdint>
#include <functional>
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

struct DecodedRequest
{
    uint8_t command{};
    uint8_t payloadType{};
};

DecodedRequest decodeRequest(std::span<const uint8_t> request)
{
    UnpackBuffer buffer(request);
    ocp::accelerator_management::MessageType messageType{};
    uint8_t instanceId = 0;
    uint8_t nvidiaMessageType = 0;
    ocp::accelerator_management::unpackHeader(
        buffer, gpu::nvidiaPciVendorId, messageType, instanceId,
        nvidiaMessageType);

    DecodedRequest decoded{};
    buffer.unpack(decoded.command);
    uint8_t dataSize = 0;
    buffer.unpack(dataSize);
    if (dataSize == 1)
    {
        buffer.unpack(decoded.payloadType);
    }
    return decoded;
}

std::array<uint8_t, gpu::supportedListBitfieldSize> bitsOf(
    std::initializer_list<uint8_t> codes)
{
    std::array<uint8_t, gpu::supportedListBitfieldSize> bits{};
    for (uint8_t code : codes)
    {
        bits[code / 8U] |= static_cast<uint8_t>(1U << (code % 8U));
    }
    return bits;
}

std::vector<uint8_t> buildSupportedListResponse(
    DiscoveryCommands command,
    const std::array<uint8_t, gpu::supportedListBitfieldSize>& bits)
{
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize +
                             gpu::supportedListBitfieldSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY));
    pack.pack(static_cast<uint8_t>(command));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(gpu::supportedListBitfieldSize));
    for (uint8_t byte : bits)
    {
        pack.pack(byte);
    }
    return buf;
}

std::vector<uint8_t> buildErrorResponse(DiscoveryCommands command)
{
    std::vector<uint8_t> buf(test_utils::errorResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY));
    pack.pack(static_cast<uint8_t>(command));
    pack.pack(static_cast<uint8_t>(ocp::accelerator_management::CompletionCode::
                                       ERR_UNSUPPORTED_COMMAND_CODE));
    pack.pack(static_cast<uint16_t>(0));
    return buf;
}

class DeviceSupportedCommandCodesTest : public MctpMockTestBase
{
  protected:
    void answerWith(std::initializer_list<uint8_t> types,
                    std::initializer_list<uint8_t> commands)
    {
        const auto typeBits = bitsOf(types);
        const auto commandBits = bitsOf(commands);
        ON_CALL(mctpMock, sendRecvMsg)
            .WillByDefault([this, typeBits,
                            commandBits](uint8_t /*eid*/,
                                         std::span<const uint8_t> request,
                                         auto callback) {
                const DecodedRequest decoded = decodeRequest(request);
                if (decoded.command ==
                    static_cast<uint8_t>(
                        DiscoveryCommands::GET_SUPPORTED_MESSAGE_TYPES))
                {
                    callback(std::error_code{},
                             buildSupportedListResponse(
                                 DiscoveryCommands::GET_SUPPORTED_MESSAGE_TYPES,
                                 typeBits));
                    return;
                }
                queriedTypes.push_back(decoded.payloadType);
                callback(std::error_code{},
                         buildSupportedListResponse(
                             DiscoveryCommands::GET_SUPPORTED_COMMAND_CODES,
                             commandBits));
            });
    }

    static std::shared_ptr<gpu::DeviceSupportedCommandCodes> makeCodes()
    {
        return std::make_shared<gpu::DeviceSupportedCommandCodes>(
            test_utils::defaultEid, requester());
    }

    static void refresh(
        const std::shared_ptr<gpu::DeviceSupportedCommandCodes>& codes)
    {
        bool done = false;
        codes->refresh([&done]() { done = true; });
        EXPECT_TRUE(done);
    }

    std::vector<uint8_t> queriedTypes;
};

TEST_F(DeviceSupportedCommandCodesTest, QueriesTheDeviceCapabilityDiscoveryType)
{
    answerWith({0, 1, 2, 3}, {static_cast<uint8_t>(
                                 DiscoveryCommands::SET_EVENT_SUBSCRIPTION)});

    const auto codes = makeCodes();
    refresh(codes);

    EXPECT_THAT(queriedTypes,
                testing::Contains(static_cast<uint8_t>(
                    gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY)));
    EXPECT_TRUE(codes->supports(DiscoveryCommands::SET_EVENT_SUBSCRIPTION));
    EXPECT_FALSE(codes->supports(DiscoveryCommands::SET_CURRENT_EVENT_SOURCES));
}

TEST_F(DeviceSupportedCommandCodesTest, QueriesEveryMessageTypeTheDeviceReports)
{
    answerWith({0, 1, 2, 3}, {0});

    refresh(makeCodes());

    EXPECT_THAT(queriedTypes, testing::UnorderedElementsAre(0, 1, 2, 3));
}

TEST_F(DeviceSupportedCommandCodesTest, SkipsMessageTypesTheDeviceDoesNotReport)
{
    answerWith({0}, {0});

    refresh(makeCodes());

    EXPECT_THAT(queriedTypes, testing::ElementsAre(0));
}

TEST_F(DeviceSupportedCommandCodesTest,
       TreatsEverythingAsSupportedOnQueryFailure)
{
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const auto codes = makeCodes();
    refresh(codes);

    EXPECT_TRUE(codes->supports(DiscoveryCommands::SET_EVENT_SUBSCRIPTION));
    EXPECT_TRUE(codes->supports(DiscoveryCommands::SET_CURRENT_EVENT_SOURCES));
}

TEST_F(DeviceSupportedCommandCodesTest,
       DropsPartialResultWhenACommandCodeQueryFails)
{
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault([](uint8_t /*eid*/, std::span<const uint8_t> request,
                          auto callback) {
            const DecodedRequest decoded = decodeRequest(request);
            if (decoded.command ==
                static_cast<uint8_t>(
                    DiscoveryCommands::GET_SUPPORTED_MESSAGE_TYPES))
            {
                callback(std::error_code{},
                         buildSupportedListResponse(
                             DiscoveryCommands::GET_SUPPORTED_MESSAGE_TYPES,
                             bitsOf({0, 1, 2, 3})));
                return;
            }
            callback(std::error_code{},
                     buildErrorResponse(
                         DiscoveryCommands::GET_SUPPORTED_COMMAND_CODES));
        });

    const auto codes = makeCodes();
    refresh(codes);

    EXPECT_TRUE(codes->supports(DiscoveryCommands::SET_EVENT_SUBSCRIPTION));
    EXPECT_TRUE(codes->supports(
        gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING));
}

TEST_F(DeviceSupportedCommandCodesTest, BeforeAnyRefreshEverythingIsSupported)
{
    const auto codes = makeCodes();

    EXPECT_TRUE(codes->supports(DiscoveryCommands::SET_EVENT_SUBSCRIPTION));
    EXPECT_TRUE(codes->supports(
        gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING));
    EXPECT_TRUE(
        codes->supports(gpu::PcieLinkCommands::QueryScalarGroupTelemetryV1));
    EXPECT_TRUE(codes->supports(
        gpu::NetworkPortCommands::GetEthernetPortTelemetryCounters));
}

TEST_F(DeviceSupportedCommandCodesTest, SupportIsScopedToItsMessageType)
{
    answerWith(
        {3}, {static_cast<uint8_t>(
                 gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING)});

    const auto codes = makeCodes();
    refresh(codes);

    EXPECT_TRUE(codes->supports(
        gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING));
    EXPECT_FALSE(
        codes->supports(gpu::PlatformEnvironmentalCommands::GET_VOLTAGE));
    EXPECT_FALSE(
        codes->supports(gpu::PcieLinkCommands::QueryScalarGroupTelemetryV1));
}

TEST_F(DeviceSupportedCommandCodesTest, AFailedRefreshKeepsThePreviousSet)
{
    answerWith(
        {3}, {static_cast<uint8_t>(
                 gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING)});

    const auto codes = makeCodes();
    refresh(codes);
    ASSERT_FALSE(
        codes->supports(gpu::PlatformEnvironmentalCommands::GET_VOLTAGE));

    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));
    refresh(codes);

    EXPECT_TRUE(codes->supports(
        gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING));
    EXPECT_FALSE(
        codes->supports(gpu::PlatformEnvironmentalCommands::GET_VOLTAGE));
}

TEST_F(DeviceSupportedCommandCodesTest,
       ASuccessfulRefreshReplacesThePreviousSet)
{
    using enum gpu::PlatformEnvironmentalCommands;

    answerWith({3}, {static_cast<uint8_t>(GET_TEMPERATURE_READING)});
    const auto codes = makeCodes();
    refresh(codes);
    ASSERT_TRUE(codes->supports(GET_TEMPERATURE_READING));
    ASSERT_FALSE(codes->supports(GET_VOLTAGE));

    answerWith({3}, {static_cast<uint8_t>(GET_VOLTAGE)});
    refresh(codes);

    EXPECT_FALSE(codes->supports(GET_TEMPERATURE_READING));
    EXPECT_TRUE(codes->supports(GET_VOLTAGE));
}

TEST_F(DeviceSupportedCommandCodesTest, ARefreshWhileOneIsInFlightIsDeferred)
{
    const auto typeBits = bitsOf({0, 1, 2, 3});
    const auto commandBits = bitsOf(
        {static_cast<uint8_t>(DiscoveryCommands::SET_EVENT_SUBSCRIPTION)});

    std::move_only_function<void(const std::error_code&,
                                 std::span<const uint8_t>)>
        parked;
    std::vector<uint8_t> parkedResponse;
    int commandCodeQueries = 0;

    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault([&](uint8_t /*eid*/, std::span<const uint8_t> request,
                           auto callback) {
            if (decodeRequest(request).command ==
                static_cast<uint8_t>(
                    DiscoveryCommands::GET_SUPPORTED_MESSAGE_TYPES))
            {
                callback(std::error_code{},
                         buildSupportedListResponse(
                             DiscoveryCommands::GET_SUPPORTED_MESSAGE_TYPES,
                             typeBits));
                return;
            }
            ++commandCodeQueries;
            if (commandCodeQueries == 1)
            {
                parkedResponse = buildSupportedListResponse(
                    DiscoveryCommands::GET_SUPPORTED_COMMAND_CODES,
                    commandBits);
                parked = std::move(callback);
                return;
            }
            callback(std::error_code{},
                     buildSupportedListResponse(
                         DiscoveryCommands::GET_SUPPORTED_COMMAND_CODES,
                         commandBits));
        });

    const auto codes = makeCodes();
    codes->refresh(nullptr);
    ASSERT_TRUE(parked);

    codes->refresh(nullptr);

    parked(std::error_code{}, parkedResponse);

    EXPECT_TRUE(codes->supports(DiscoveryCommands::SET_EVENT_SUBSCRIPTION));
    EXPECT_FALSE(codes->supports(DiscoveryCommands::SET_CURRENT_EVENT_SOURCES));
}

} // namespace
