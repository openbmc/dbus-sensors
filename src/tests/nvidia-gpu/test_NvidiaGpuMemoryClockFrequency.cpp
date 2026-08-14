/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaGpuMemoryClockFrequency.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

const std::string dramIface = "xyz.openbmc_project.Inventory.Item.Dimm";

std::vector<uint8_t> buildClockFreqResponse(uint32_t mhz)
{
    return test_utils::buildPlatformEnvSuccessResponse(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY, mhz);
}

std::vector<uint8_t> buildErrorResponse()
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY,
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::ERROR),
        0x1234);
}

class NvidiaGpuMemoryClockFrequencyTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<sdbusplus::asio::dbus_interface> createDramInterface(
        const std::string& name)
    {
        const std::string path =
            "/xyz/openbmc_project/inventory/" + name + "_DRAM_0";
        auto iface = objects().add_interface(path, dramIface);
        iface->register_property("MemoryConfiguredSpeedInMhz", uint16_t{0});
        iface->initialize();
        return iface;
    }

    static std::shared_ptr<NvidiaGpuMemoryClockFrequency> createMemFreq(
        const std::string& name = "GPU_MEM",
        uint8_t eid = test_utils::defaultEid)
    {
        auto iface = createDramInterface(name);
        return std::make_shared<NvidiaGpuMemoryClockFrequency>(
            requester(), name, eid, iface);
    }

    static std::string dramPath(const std::string& name)
    {
        return "/xyz/openbmc_project/inventory/" + name + "_DRAM_0";
    }
};

TEST_F(NvidiaGpuMemoryClockFrequencyTest, ConstructorDoesNotCrash)
{
    auto metric = createMemFreq("mem_ctor");
    ASSERT_NE(metric, nullptr);
}

TEST_F(NvidiaGpuMemoryClockFrequencyTest, UpdateSuccessSetsMemorySpeed)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildClockFreqResponse(2619)));

    auto metric = createMemFreq("mem_succ");
    metric->update();

    EXPECT_EQ(getProperty<uint16_t>(dramPath("mem_succ"), dramIface,
                                    "MemoryConfiguredSpeedInMhz"),
              2619);
}

TEST_F(NvidiaGpuMemoryClockFrequencyTest, UpdateTruncatesTo16Bits)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildClockFreqResponse(65537)));

    auto metric = createMemFreq("mem_trunc");
    metric->update();

    EXPECT_EQ(getProperty<uint16_t>(dramPath("mem_trunc"), dramIface,
                                    "MemoryConfiguredSpeedInMhz"),
              1);
}

TEST_F(NvidiaGpuMemoryClockFrequencyTest, UpdateSendsRequest)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    auto metric = createMemFreq("mem_sends");
    metric->update();
}

TEST_F(NvidiaGpuMemoryClockFrequencyTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, {}));

    auto metric = createMemFreq("mem_eid", testEid);
    metric->update();
}

TEST_F(NvidiaGpuMemoryClockFrequencyTest, UpdateVerifiesRequestEncoding)
{
    // Copy the request bytes before completing the call: the reqMsg span is
    // a view into caller-owned memory, valid only during the call.
    std::vector<uint8_t> lastRequest;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce([&](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                      auto callback) {
            lastRequest.assign(reqMsg.begin(), reqMsg.end());
            callback(std::error_code{}, std::span<const uint8_t>{});
        });

    auto metric = createMemFreq("mem_enc");
    metric->update();

    ASSERT_FALSE(lastRequest.empty());

    UnpackBuffer unpack(lastRequest);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId,
                  msgType),
              0);

    uint8_t command = 0;
    uint8_t dataSize = 0;
    uint8_t clockType = 0;
    unpack.unpack(command);
    unpack.unpack(dataSize);
    unpack.unpack(clockType);
    EXPECT_EQ(
        command,
        static_cast<uint8_t>(
            gpu::PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY));
    EXPECT_EQ(clockType, static_cast<uint8_t>(gpu::ClockType::MEMORY_CLOCK));
    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaGpuMemoryClockFrequencyTest, UpdateMctpTransportErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    auto metric = createMemFreq("mem_mctp_err");
    EXPECT_NO_THROW(metric->update());
}

TEST_F(NvidiaGpuMemoryClockFrequencyTest, UpdateDecodeErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildErrorResponse()));

    auto metric = createMemFreq("mem_dec_err");
    EXPECT_NO_THROW(metric->update());
}

TEST_F(NvidiaGpuMemoryClockFrequencyTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    auto metric = createMemFreq("mem_empty");
    EXPECT_NO_THROW(metric->update());
}

TEST_F(NvidiaGpuMemoryClockFrequencyTest, UpdateTinyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, {0x00, 0x01}));

    auto metric = createMemFreq("mem_tiny");
    EXPECT_NO_THROW(metric->update());
}

TEST_F(NvidiaGpuMemoryClockFrequencyTest, UpdateMctpTransportError)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    auto metric = createMemFreq("mem_mctp_err_val");
    metric->update();

    // A transport error must leave the speed at its default (0).
    EXPECT_EQ(getProperty<uint16_t>(dramPath("mem_mctp_err_val"), dramIface,
                                    "MemoryConfiguredSpeedInMhz"),
              0);
}

TEST_F(NvidiaGpuMemoryClockFrequencyTest,
       UpdateSuccessThenErrorKeepsPreviousValue)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildClockFreqResponse(2619)))
        .WillOnce(mock_mctp::respondWith({}, buildErrorResponse()));

    auto metric = createMemFreq("mem_keep");

    metric->update();
    EXPECT_EQ(getProperty<uint16_t>(dramPath("mem_keep"), dramIface,
                                    "MemoryConfiguredSpeedInMhz"),
              2619);

    // A subsequent error response must not overwrite the last good value.
    metric->update();
    EXPECT_EQ(getProperty<uint16_t>(dramPath("mem_keep"), dramIface,
                                    "MemoryConfiguredSpeedInMhz"),
              2619);
}

} // namespace
