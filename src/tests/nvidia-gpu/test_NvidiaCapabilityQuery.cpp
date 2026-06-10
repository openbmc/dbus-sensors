/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaCapabilityQuery.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

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

class NvidiaCapabilityQueryTest : public MctpMockTestBase
{
  protected:
    // Answers GetSupportedMessageTypes with `types` and every
    // GetSupportedCommandCodes with `commands`, recording the message type
    // each command-code query asked about.
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

    static gpu::DeviceCapabilities run()
    {
        gpu::DeviceCapabilities caps{};
        auto query = std::make_shared<CapabilityQuery>(
            test_utils::defaultEid, requester(),
            [&caps](const gpu::DeviceCapabilities& result) { caps = result; });
        query->start();
        return caps;
    }

    std::vector<uint8_t> queriedTypes;
};

TEST_F(NvidiaCapabilityQueryTest, QueriesTheDeviceCapabilityDiscoveryType)
{
    answerWith({0, 1, 2, 3}, {static_cast<uint8_t>(
                                 DiscoveryCommands::SET_EVENT_SUBSCRIPTION)});

    const gpu::DeviceCapabilities caps = run();

    EXPECT_THAT(queriedTypes,
                testing::Contains(static_cast<uint8_t>(
                    gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY)));
    EXPECT_TRUE(caps.queried);
    EXPECT_TRUE(caps.supports(DiscoveryCommands::SET_EVENT_SUBSCRIPTION));
    EXPECT_FALSE(caps.supports(DiscoveryCommands::SET_CURRENT_EVENT_SOURCES));
}

TEST_F(NvidiaCapabilityQueryTest, QueriesEveryMessageTypeTheDeviceReports)
{
    answerWith({0, 1, 2, 3}, {0});

    run();

    EXPECT_THAT(queriedTypes, testing::UnorderedElementsAre(0, 1, 2, 3));
}

TEST_F(NvidiaCapabilityQueryTest, SkipsMessageTypesTheDeviceDoesNotReport)
{
    answerWith({0}, {0});

    run();

    EXPECT_THAT(queriedTypes, testing::ElementsAre(0));
}

TEST_F(NvidiaCapabilityQueryTest, TreatsEverythingAsSupportedOnQueryFailure)
{
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const gpu::DeviceCapabilities caps = run();

    EXPECT_FALSE(caps.queried);
    EXPECT_TRUE(caps.supports(DiscoveryCommands::SET_EVENT_SUBSCRIPTION));
}

TEST_F(NvidiaCapabilityQueryTest, DropsPartialResultWhenACommandCodeQueryFails)
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

    const gpu::DeviceCapabilities caps = run();

    EXPECT_FALSE(caps.queried);
    EXPECT_TRUE(caps.commands.empty());
}

} // namespace
