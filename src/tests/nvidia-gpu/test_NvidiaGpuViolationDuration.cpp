/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaGpuViolationDuration.hpp"
#include "NvidiaLongRunningHandler.hpp"
#include "OcpMctpVdm.hpp"
#include "SerialQueue.hpp"
#include "TestUtils.hpp"

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

constexpr const char* metricIface = "xyz.openbmc_project.Metric.Value";

// Build a GET_VIOLATION_DURATION response with the given completion code:
//   common response header + 4 x uint64_t (hw, globalSw, power, thermal) in ns
std::vector<uint8_t> buildViolationResponse(
    ocp::accelerator_management::CompletionCode cc, uint64_t hw,
    uint64_t globalSw, uint64_t power, uint64_t thermal)
{
    const uint16_t dataSize = sizeof(uint64_t) * 4;
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + dataSize, 0);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_VIOLATION_DURATION));
    pack.pack(static_cast<uint8_t>(cc));
    pack.pack(static_cast<uint16_t>(0)); // reasonCode
    pack.pack(dataSize);
    pack.pack(hw);
    pack.pack(globalSw);
    pack.pack(power);
    pack.pack(thermal);
    EXPECT_EQ(pack.getError(), 0);
    return buf;
}

std::vector<uint8_t> buildErrorResponse()
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_VIOLATION_DURATION,
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::ERROR),
        0x1234);
}

class NvidiaGpuViolationDurationTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaGpuViolationDuration> createViolation(
        const std::string& name = "GPU_VIOL",
        uint8_t eid = test_utils::defaultEid)
    {
        auto queue = std::make_shared<SerialQueue>(ioContext());
        auto handler =
            std::make_shared<NvidiaLongRunningResponseHandler>(ioContext());
        return std::make_shared<NvidiaGpuViolationDuration>(
            requester(), objects(), name, eid, queue, handler);
    }

    static std::string powerPath(const std::string& name)
    {
        return "/xyz/openbmc_project/metric/gpu_" + name +
               "/power_limit_throttle_duration";
    }

    static std::string thermalPath(const std::string& name)
    {
        return "/xyz/openbmc_project/metric/gpu_" + name +
               "/thermal_limit_throttle_duration";
    }
};

TEST_F(NvidiaGpuViolationDurationTest, ConstructorDoesNotCrash)
{
    auto viol = createViolation("viol_ctor");
    ASSERT_NE(viol, nullptr);
}

TEST_F(NvidiaGpuViolationDurationTest, UpdateImmediateSuccessSetsValues)
{
    // power = 2e9 ns -> 2.0 s; thermal = 5e8 ns -> 0.5 s
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildViolationResponse(
                    ocp::accelerator_management::CompletionCode::SUCCESS, 100,
                    200, 2'000'000'000ULL, 500'000'000ULL)));

    auto viol = createViolation("viol_succ");
    viol->update();

    EXPECT_DOUBLE_EQ(
        getProperty<double>(powerPath("viol_succ"), metricIface, "Value"), 2.0);
    EXPECT_DOUBLE_EQ(
        getProperty<double>(thermalPath("viol_succ"), metricIface, "Value"),
        0.5);
}

TEST_F(NvidiaGpuViolationDurationTest, UpdateSendsRequest)
{
    // WillOnce cardinality asserts the request was actually sent.
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    auto viol = createViolation("viol_sends");
    viol->update();
}

TEST_F(NvidiaGpuViolationDurationTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, {}));

    auto viol = createViolation("viol_eid", testEid);
    viol->update();
}

TEST_F(NvidiaGpuViolationDurationTest, UpdateVerifiesRequestEncoding)
{
    // Copy the request bytes before completing the call: the reqMsg span is
    // a view into caller-owned memory, valid only during the call.
    std::vector<uint8_t> lastRequest;
    const std::vector<uint8_t> response;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce([&](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                      auto callback) {
            lastRequest.assign(reqMsg.begin(), reqMsg.end());
            callback(std::error_code{}, response);
        });

    auto viol = createViolation("viol_enc");
    viol->update();

    ASSERT_FALSE(lastRequest.empty());

    UnpackBuffer unpack(lastRequest);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId,
                  msgType),
              0);
    EXPECT_EQ(msgType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    uint8_t command = 0;
    unpack.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::GET_VIOLATION_DURATION));
    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaGpuViolationDurationTest,
       UpdateAcceptedRegistersLongRunningNoCrash)
{
    // WillOnce cardinality asserts the request was actually sent.
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildViolationResponse(
                    ocp::accelerator_management::CompletionCode::ACCEPTED, 0, 0,
                    0, 0)));

    auto viol = createViolation("viol_accepted");
    EXPECT_NO_THROW(viol->update());
}

TEST_F(NvidiaGpuViolationDurationTest, UpdateMctpTransportErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    auto viol = createViolation("viol_mctp_err");
    EXPECT_NO_THROW(viol->update());
}

TEST_F(NvidiaGpuViolationDurationTest, UpdateDecodeErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildErrorResponse()));

    auto viol = createViolation("viol_dec_err");
    EXPECT_NO_THROW(viol->update());
}

TEST_F(NvidiaGpuViolationDurationTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    auto viol = createViolation("viol_empty");
    EXPECT_NO_THROW(viol->update());
}

TEST_F(NvidiaGpuViolationDurationTest, UpdateTinyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, {0x00, 0x01}));

    auto viol = createViolation("viol_tiny");
    EXPECT_NO_THROW(viol->update());
}

} // namespace
