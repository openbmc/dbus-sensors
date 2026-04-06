/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuClockSpeedControl.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <MessagePackUnpackUtils.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/exception.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

namespace
{

constexpr uint64_t mhzToHz = 1'000'000;

using Association = std::tuple<std::string, std::string, std::string>;

// Build a successful GET_CLOCK_LIMIT response:
//   common response header + 4 x uint32_t
//   (requestedMin, requestedMax, presentMin, presentMax)
std::vector<uint8_t> buildClockLimitResponse(uint32_t reqMin, uint32_t reqMax,
                                             uint32_t presMin, uint32_t presMax)
{
    const uint16_t dataSize = sizeof(uint32_t) * 4;
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + dataSize, 0);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_CLOCK_LIMIT));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reasonCode
    pack.pack(dataSize);
    pack.pack(reqMin);
    pack.pack(reqMax);
    pack.pack(presMin);
    pack.pack(presMax);
    EXPECT_EQ(pack.getError(), 0);
    return buf;
}

std::vector<uint8_t> buildErrorResponse()
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_CLOCK_LIMIT,
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::ERROR),
        0x1234);
}

class NvidiaGpuClockSpeedControlTestBase : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<sdbusplus::asio::dbus_interface>
        createControlInterface(const std::string& name)
    {
        const std::string path =
            "/xyz/openbmc_project/control/clock/" + name + "_iface";
        auto iface = objectServer->add_interface(
            path, "xyz.openbmc_project.Control.Mode.ClockSpeed");
        iface->register_property("PresentSpeedLimitMaxHz", uint64_t{0});
        iface->register_property("PresentSpeedLimitMinHz", uint64_t{0});
        iface->register_property("RequestedSpeedLimitMaxHz", uint64_t{0});
        iface->register_property("RequestedSpeedLimitMinHz", uint64_t{0});
        iface->initialize();
        return iface;
    }

    static std::shared_ptr<NvidiaGpuClockSpeedControl> createControl(
        const std::string& name = "GPU_CLK",
        uint8_t eid = test_utils::defaultEid)
    {
        auto iface = createControlInterface(name);
        const std::string invPath = "/xyz/openbmc_project/inventory/" + name;
        return std::make_shared<NvidiaGpuClockSpeedControl>(
            *objectServer, name, invPath, *mctpRequester, eid, iface);
    }
};

TEST_F(NvidiaGpuClockSpeedControlTestBase, ConstructorDoesNotCrash)
{
    auto ctrl = createControl("clk_ctor");
    ASSERT_NE(ctrl, nullptr);
}

TEST_F(NvidiaGpuClockSpeedControlTestBase, UpdateSuccessSetsProperties)
{
    mock_mctp::setNextResponse({},
                               buildClockLimitResponse(100, 2000, 200, 1800));
    auto ctrl = createControl("clk_succ");
    ctrl->update();

    const std::string path =
        "/xyz/openbmc_project/control/clock/clk_succ_iface";
    const std::string iface = "xyz.openbmc_project.Control.Mode.ClockSpeed";
    EXPECT_EQ(getProperty<uint64_t>(path, iface, "RequestedSpeedLimitMinHz"),
              100 * mhzToHz);
    EXPECT_EQ(getProperty<uint64_t>(path, iface, "RequestedSpeedLimitMaxHz"),
              2000 * mhzToHz);
    EXPECT_EQ(getProperty<uint64_t>(path, iface, "PresentSpeedLimitMinHz"),
              200 * mhzToHz);
    EXPECT_EQ(getProperty<uint64_t>(path, iface, "PresentSpeedLimitMaxHz"),
              1800 * mhzToHz);
}

TEST_F(NvidiaGpuClockSpeedControlTestBase, UpdateSendsRequest)
{
    mock_mctp::setNextResponse({}, {});
    auto ctrl = createControl("clk_sends");
    ctrl->update();
    EXPECT_GE(mock_mctp::getRequestCount(), 1U);
}

TEST_F(NvidiaGpuClockSpeedControlTestBase, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    mock_mctp::setNextResponse({}, {});
    auto ctrl = createControl("clk_eid", testEid);
    ctrl->update();
    EXPECT_EQ(mock_mctp::getLastEid(), testEid);
}

TEST_F(NvidiaGpuClockSpeedControlTestBase, UpdateVerifiesRequestEncoding)
{
    mock_mctp::setNextResponse({}, {});
    auto ctrl = createControl("clk_enc");
    ctrl->update();

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
    EXPECT_EQ(ocpMsgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(msgType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    uint8_t command = 0;
    uint8_t dataSize = 0;
    uint8_t clockType = 0;
    unpack.unpack(command);
    unpack.unpack(dataSize);
    unpack.unpack(clockType);
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::GET_CLOCK_LIMIT));
    EXPECT_EQ(clockType, static_cast<uint8_t>(gpu::ClockType::GRAPHICS_CLOCK));
    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaGpuClockSpeedControlTestBase, UpdateMctpTransportErrorNoCrash)
{
    mock_mctp::setNextResponse(std::make_error_code(std::errc::timed_out), {});
    auto ctrl = createControl("clk_mctp_err");
    EXPECT_NO_THROW(ctrl->update());
}

TEST_F(NvidiaGpuClockSpeedControlTestBase, UpdateDecodeErrorNoCrash)
{
    mock_mctp::setNextResponse({}, buildErrorResponse());
    auto ctrl = createControl("clk_dec_err");
    EXPECT_NO_THROW(ctrl->update());
}

TEST_F(NvidiaGpuClockSpeedControlTestBase, UpdateEmptyBufferNoCrash)
{
    mock_mctp::setNextResponse({}, {});
    auto ctrl = createControl("clk_empty");
    EXPECT_NO_THROW(ctrl->update());
}

TEST_F(NvidiaGpuClockSpeedControlTestBase, UpdateTinyBufferNoCrash)
{
    mock_mctp::setNextResponse({}, {0x00, 0x01});
    auto ctrl = createControl("clk_tiny");
    EXPECT_NO_THROW(ctrl->update());
}

TEST_F(NvidiaGpuClockSpeedControlTestBase, DestructorRemovesInterface)
{
    const std::string name = "clk_dtor";
    const std::string assocPath =
        "/xyz/openbmc_project/control/clock/" + name + "_iface";
    {
        auto ctrl = createControl(name);
        ASSERT_NE(ctrl, nullptr);
        EXPECT_NO_THROW(getProperty<std::vector<Association>>(
            assocPath, "xyz.openbmc_project.Association.Definitions",
            "Associations"));
    }
    EXPECT_THROW(getProperty<std::vector<Association>>(
                     assocPath, "xyz.openbmc_project.Association.Definitions",
                     "Associations"),
                 sdbusplus::exception_t);
}

} // namespace
