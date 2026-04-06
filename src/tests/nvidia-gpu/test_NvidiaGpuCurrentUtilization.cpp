/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuCurrentUtilization.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaLongRunningHandler.hpp"
#include "OcpMctpVdm.hpp"
#include "SerialQueue.hpp"
#include "TestUtils.hpp"

#include <MessagePackUnpackUtils.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

namespace
{

const std::string metricIface = "xyz.openbmc_project.Metric.Value";

// Build a GET_CURRENT_UTILIZATION response with the given completion code:
//   common response header + 2 x uint32_t (gpuUtil, memUtil)
std::vector<uint8_t> buildUtilizationResponse(
    ocp::accelerator_management::CompletionCode cc, uint32_t gpuUtil,
    uint32_t memUtil)
{
    const uint16_t dataSize = sizeof(uint32_t) * 2;
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + dataSize, 0);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION));
    pack.pack(static_cast<uint8_t>(cc));
    pack.pack(static_cast<uint16_t>(0)); // reasonCode
    pack.pack(dataSize);
    pack.pack(gpuUtil);
    pack.pack(memUtil);
    EXPECT_EQ(pack.getError(), 0);
    return buf;
}

std::vector<uint8_t> buildErrorResponse()
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION,
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::ERROR),
        0x1234);
}

class NvidiaGpuCurrentUtilizationTestBase : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaGpuCurrentUtilization> createUtil(
        const std::string& name = "GPU_UTIL",
        uint8_t eid = test_utils::defaultEid)
    {
        auto queue = std::make_shared<SerialQueue>(io);
        auto handler = std::make_shared<NvidiaLongRunningResponseHandler>(io);
        return std::make_shared<NvidiaGpuCurrentUtilization>(
            *mctpRequester, *objectServer, name, eid, queue, handler);
    }

    static std::string metricDbusPath(const std::string& name)
    {
        return "/xyz/openbmc_project/metric/gpu_" + name +
               "/processor_bandwidth";
    }
};

TEST_F(NvidiaGpuCurrentUtilizationTestBase, ConstructorDoesNotCrash)
{
    auto util = createUtil("util_ctor");
    ASSERT_NE(util, nullptr);
}

TEST_F(NvidiaGpuCurrentUtilizationTestBase, UpdateImmediateSuccessSetsValue)
{
    mock_mctp::setNextResponse(
        {}, buildUtilizationResponse(
                ocp::accelerator_management::CompletionCode::SUCCESS, 85, 40));
    auto util = createUtil("util_succ");
    util->update();

    // Only the GPU utilization (first value) is published.
    EXPECT_DOUBLE_EQ(
        getProperty<double>(metricDbusPath("util_succ"), metricIface, "Value"),
        85.0);
}

TEST_F(NvidiaGpuCurrentUtilizationTestBase, UpdateSendsRequest)
{
    mock_mctp::setNextResponse({}, {});
    auto util = createUtil("util_sends");
    util->update();
    EXPECT_GE(mock_mctp::getRequestCount(), 1U);
}

TEST_F(NvidiaGpuCurrentUtilizationTestBase, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    mock_mctp::setNextResponse({}, {});
    auto util = createUtil("util_eid", testEid);
    util->update();
    EXPECT_EQ(mock_mctp::getLastEid(), testEid);
}

TEST_F(NvidiaGpuCurrentUtilizationTestBase, UpdateVerifiesRequestEncoding)
{
    mock_mctp::setNextResponse({}, {});
    auto util = createUtil("util_enc");
    util->update();

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
    EXPECT_EQ(msgType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    uint8_t command = 0;
    unpack.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION));
    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaGpuCurrentUtilizationTestBase,
       UpdateAcceptedRegistersLongRunningNoCrash)
{
    mock_mctp::setNextResponse(
        {}, buildUtilizationResponse(
                ocp::accelerator_management::CompletionCode::ACCEPTED, 0, 0));
    auto util = createUtil("util_accepted");
    EXPECT_NO_THROW(util->update());
    EXPECT_GE(mock_mctp::getRequestCount(), 1U);
}

TEST_F(NvidiaGpuCurrentUtilizationTestBase, UpdateMctpTransportErrorNoCrash)
{
    mock_mctp::setNextResponse(std::make_error_code(std::errc::timed_out), {});
    auto util = createUtil("util_mctp_err");
    EXPECT_NO_THROW(util->update());
}

TEST_F(NvidiaGpuCurrentUtilizationTestBase, UpdateDecodeErrorNoCrash)
{
    mock_mctp::setNextResponse({}, buildErrorResponse());
    auto util = createUtil("util_dec_err");
    EXPECT_NO_THROW(util->update());
}

TEST_F(NvidiaGpuCurrentUtilizationTestBase, UpdateEmptyBufferNoCrash)
{
    mock_mctp::setNextResponse({}, {});
    auto util = createUtil("util_empty");
    EXPECT_NO_THROW(util->update());
}

TEST_F(NvidiaGpuCurrentUtilizationTestBase, UpdateTinyBufferNoCrash)
{
    mock_mctp::setNextResponse({}, {0x00, 0x01});
    auto util = createUtil("util_tiny");
    EXPECT_NO_THROW(util->update());
}

} // namespace
