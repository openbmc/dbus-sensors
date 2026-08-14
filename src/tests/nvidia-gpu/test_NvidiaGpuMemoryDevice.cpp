/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaGpuMemoryDevice.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <sdbusplus/exception.hpp>

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

const std::string eccIface = "xyz.openbmc_project.Memory.MemoryECC";

// Build a successful GET_ECC_ERROR_COUNTS response:
//   common response header + uint16 flags + 5 x uint32_t
std::vector<uint8_t> buildEccResponse(uint16_t flags, uint32_t sramCorr,
                                      uint32_t sramSecded, uint32_t sramParity,
                                      uint32_t dramCorr, uint32_t dramUncorr)
{
    const uint16_t dataSize = sizeof(uint16_t) + (sizeof(uint32_t) * 5);
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + dataSize, 0);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_ECC_ERROR_COUNTS));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reasonCode
    pack.pack(dataSize);
    pack.pack(flags);
    pack.pack(sramCorr);
    pack.pack(sramSecded);
    pack.pack(sramParity);
    pack.pack(dramCorr);
    pack.pack(dramUncorr);
    EXPECT_EQ(pack.getError(), 0);
    return buf;
}

std::vector<uint8_t> buildErrorResponse()
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_ECC_ERROR_COUNTS,
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::ERROR),
        0x1234);
}

class NvidiaGpuMemoryDeviceTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaGpuMemoryDevice> createDevice(
        const std::string& name = "GPU_MEM_DEV",
        uint8_t eid = test_utils::defaultEid)
    {
        return std::make_shared<NvidiaGpuMemoryDevice>(bus(), requester(), name,
                                                       eid, objects());
    }

    static std::string gpuPath(const std::string& name)
    {
        return "/xyz/openbmc_project/inventory/" + name;
    }

    static std::string dramPath(const std::string& name)
    {
        return "/xyz/openbmc_project/inventory/" + name + "_DRAM_0";
    }
};

TEST_F(NvidiaGpuMemoryDeviceTest, ConstructorDoesNotCrash)
{
    auto dev = createDevice("memdev_ctor");
    ASSERT_NE(dev, nullptr);
}

TEST_F(NvidiaGpuMemoryDeviceTest, UpdateSuccessSetsEccCounts)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildEccResponse(0, 5, 2, 3, 7, 9)));

    auto dev = createDevice("memdev_succ");
    dev->update();

    // SRAM: ceCount = sramCorrected (5); ueCount = secded + parity (2 + 3)
    EXPECT_EQ(getProperty<int64_t>(gpuPath("memdev_succ"), eccIface, "ceCount"),
              5);
    EXPECT_EQ(getProperty<int64_t>(gpuPath("memdev_succ"), eccIface, "ueCount"),
              5);

    // DRAM: ceCount = dramCorrected (7); ueCount = dramUncorrected (9)
    EXPECT_EQ(
        getProperty<int64_t>(dramPath("memdev_succ"), eccIface, "ceCount"), 7);
    EXPECT_EQ(
        getProperty<int64_t>(dramPath("memdev_succ"), eccIface, "ueCount"), 9);
}

TEST_F(NvidiaGpuMemoryDeviceTest, UpdateSendsRequest)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    auto dev = createDevice("memdev_sends");
    dev->update();
}

TEST_F(NvidiaGpuMemoryDeviceTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, {}));

    auto dev = createDevice("memdev_eid", testEid);
    dev->update();
}

TEST_F(NvidiaGpuMemoryDeviceTest, UpdateVerifiesRequestEncoding)
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

    auto dev = createDevice("memdev_enc");
    dev->update();

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
    unpack.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::GET_ECC_ERROR_COUNTS));
    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaGpuMemoryDeviceTest, UpdateMctpTransportErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    auto dev = createDevice("memdev_mctp_err");
    EXPECT_NO_THROW(dev->update());
}

TEST_F(NvidiaGpuMemoryDeviceTest, UpdateDecodeErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildErrorResponse()));

    auto dev = createDevice("memdev_dec_err");
    EXPECT_NO_THROW(dev->update());
}

TEST_F(NvidiaGpuMemoryDeviceTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    auto dev = createDevice("memdev_empty");
    EXPECT_NO_THROW(dev->update());
}

TEST_F(NvidiaGpuMemoryDeviceTest, UpdateTinyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, {0x00, 0x01}));

    auto dev = createDevice("memdev_tiny");
    EXPECT_NO_THROW(dev->update());
}

TEST_F(NvidiaGpuMemoryDeviceTest, DestructorRemovesInterfaces)
{
    const std::string name = "memdev_dtor";
    {
        auto dev = createDevice(name);
        ASSERT_NE(dev, nullptr);
        EXPECT_NO_THROW(
            getProperty<int64_t>(gpuPath(name), eccIface, "ceCount"));
        EXPECT_NO_THROW(
            getProperty<int64_t>(dramPath(name), eccIface, "ceCount"));
    }
    drainPendingAsync();
    EXPECT_THROW(getProperty<int64_t>(gpuPath(name), eccIface, "ceCount"),
                 sdbusplus::exception_t);
    EXPECT_THROW(getProperty<int64_t>(dramPath(name), eccIface, "ceCount"),
                 sdbusplus::exception_t);
}

} // namespace
