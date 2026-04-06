/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaGpuMemoryClockFrequency.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <MessagePackUnpackUtils.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

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

class NvidiaGpuMemoryClockFrequencyTestBase : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<sdbusplus::asio::dbus_interface> createDramInterface(
        const std::string& name)
    {
        const std::string path =
            "/xyz/openbmc_project/inventory/" + name + "_DRAM_0";
        auto iface = objectServer->add_interface(path, dramIface);
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
            *mctpRequester, name, eid, iface);
    }

    static std::string dramPath(const std::string& name)
    {
        return "/xyz/openbmc_project/inventory/" + name + "_DRAM_0";
    }
};

TEST_F(NvidiaGpuMemoryClockFrequencyTestBase, ConstructorDoesNotCrash)
{
    auto metric = createMemFreq("mem_ctor");
    ASSERT_NE(metric, nullptr);
}

TEST_F(NvidiaGpuMemoryClockFrequencyTestBase, UpdateSuccessSetsMemorySpeed)
{
    mock_mctp::setNextResponse({}, buildClockFreqResponse(2619));
    auto metric = createMemFreq("mem_succ");
    metric->update();

    EXPECT_EQ(getProperty<uint16_t>(dramPath("mem_succ"), dramIface,
                                    "MemoryConfiguredSpeedInMhz"),
              2619);
}

TEST_F(NvidiaGpuMemoryClockFrequencyTestBase, UpdateTruncatesTo16Bits)
{
    mock_mctp::setNextResponse({}, buildClockFreqResponse(65537));
    auto metric = createMemFreq("mem_trunc");
    metric->update();

    EXPECT_EQ(getProperty<uint16_t>(dramPath("mem_trunc"), dramIface,
                                    "MemoryConfiguredSpeedInMhz"),
              1);
}

TEST_F(NvidiaGpuMemoryClockFrequencyTestBase, UpdateSendsRequest)
{
    mock_mctp::setNextResponse({}, {});
    auto metric = createMemFreq("mem_sends");
    metric->update();
    EXPECT_GE(mock_mctp::getRequestCount(), 1U);
}

TEST_F(NvidiaGpuMemoryClockFrequencyTestBase, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    mock_mctp::setNextResponse({}, {});
    auto metric = createMemFreq("mem_eid", testEid);
    metric->update();
    EXPECT_EQ(mock_mctp::getLastEid(), testEid);
}

TEST_F(NvidiaGpuMemoryClockFrequencyTestBase, UpdateVerifiesRequestEncoding)
{
    mock_mctp::setNextResponse({}, {});
    auto metric = createMemFreq("mem_enc");
    metric->update();

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

TEST_F(NvidiaGpuMemoryClockFrequencyTestBase, UpdateMctpTransportErrorNoCrash)
{
    mock_mctp::setNextResponse(std::make_error_code(std::errc::timed_out), {});
    auto metric = createMemFreq("mem_mctp_err");
    EXPECT_NO_THROW(metric->update());
}

TEST_F(NvidiaGpuMemoryClockFrequencyTestBase, UpdateDecodeErrorNoCrash)
{
    mock_mctp::setNextResponse({}, buildErrorResponse());
    auto metric = createMemFreq("mem_dec_err");
    EXPECT_NO_THROW(metric->update());
}

TEST_F(NvidiaGpuMemoryClockFrequencyTestBase, UpdateEmptyBufferNoCrash)
{
    mock_mctp::setNextResponse({}, {});
    auto metric = createMemFreq("mem_empty");
    EXPECT_NO_THROW(metric->update());
}

TEST_F(NvidiaGpuMemoryClockFrequencyTestBase, UpdateTinyBufferNoCrash)
{
    mock_mctp::setNextResponse({}, {0x00, 0x01});
    auto metric = createMemFreq("mem_tiny");
    EXPECT_NO_THROW(metric->update());
}

} // namespace
