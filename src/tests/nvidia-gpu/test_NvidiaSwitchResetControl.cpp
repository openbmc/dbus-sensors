/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaSwitchResetControl.hpp"
#include "OcpMctpVdm.hpp"

#include <boost/system/error_code.hpp>
#include <sdbusplus/asio/property.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr uint8_t testEid = 10;

constexpr const char* resetIfaceName = "xyz.openbmc_project.Control.Reset";
constexpr const char* forceRestartValue =
    "xyz.openbmc_project.Control.Reset.ResetTypes.ForceRestart";
constexpr const char* noneValue =
    "xyz.openbmc_project.Control.Reset.ResetTypes.None";

std::string resetPath(const std::string& name)
{
    return "/xyz/openbmc_project/control/reset/" + name;
}

// A SUCCESS ResetNetworkDevice (DIAGNOSTICS) response: common response header
// followed by the completion code and reason code.
std::vector<uint8_t> buildResetSuccessResponse()
{
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::DIAGNOSTICS));
    pack.pack(
        static_cast<uint8_t>(gpu::DiagnosticsCommands::ResetNetworkDevice));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(uint16_t{0}); // reasonCode
    pack.pack(uint16_t{0}); // dataSize
    EXPECT_EQ(pack.getError(), 0);
    return buf;
}

class NvidiaSwitchResetControlTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaSwitchResetControl> createControl(
        const std::string& name, uint8_t eid = testEid)
    {
        return std::make_shared<NvidiaSwitchResetControl>(
            objects(), requester(), name,
            "/xyz/openbmc_project/inventory/" + name, eid);
    }

    // The reset D-Bus interface is private to the control, so drive the
    // PendingReset setter over D-Bus. Returns the error from the set, which
    // is non-empty when the set handler rejects the write.
    static boost::system::error_code setPendingReset(const std::string& name,
                                                     const std::string& value)
    {
        auto ec = std::make_shared<std::optional<boost::system::error_code>>();
        sdbusplus::asio::setProperty<std::string>(
            *bus(), bus()->get_unique_name(), resetPath(name), resetIfaceName,
            "PendingReset", std::string(value),
            [ec](boost::system::error_code e) { *ec = e; });
        pumpIoUntil([ec] { return ec->has_value(); }, std::chrono::seconds{5});
        return ec->value_or(boost::system::error_code{});
    }
};

TEST_F(NvidiaSwitchResetControlTest, CreatesResetObject)
{
    auto ctrl = createControl("cx9_create");

    EXPECT_EQ(getProperty<std::string>(resetPath("cx9_create"), resetIfaceName,
                                       "PendingReset"),
              noneValue);

    auto supported = getProperty<std::vector<std::string>>(
        resetPath("cx9_create"), resetIfaceName, "SupportedResetTypes");
    ASSERT_EQ(supported.size(), 1U);
    EXPECT_EQ(supported[0], forceRestartValue);
}

TEST_F(NvidiaSwitchResetControlTest, ForceRestartSendsResetAndClearsPending)
{
    std::vector<uint8_t> lastRequest;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce([&](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                      auto callback) {
            lastRequest.assign(reqMsg.begin(), reqMsg.end());
            callback(std::error_code{}, buildResetSuccessResponse());
        });

    auto ctrl = createControl("cx9_reset");
    EXPECT_FALSE(setPendingReset("cx9_reset", forceRestartValue));

    // The reset completes synchronously, so PendingReset is cleared to None.
    EXPECT_EQ(getProperty<std::string>(resetPath("cx9_reset"), resetIfaceName,
                                       "PendingReset"),
              noneValue);

    // The request is a DIAGNOSTICS ResetNetworkDevice command.
    ASSERT_FALSE(lastRequest.empty());
    UnpackBuffer unpack(lastRequest);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId,
                  msgType),
              0);
    EXPECT_EQ(ocpMsgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(msgType, static_cast<uint8_t>(gpu::MessageType::DIAGNOSTICS));
    uint8_t command = 0;
    unpack.unpack(command);
    EXPECT_EQ(command, static_cast<uint8_t>(
                           gpu::DiagnosticsCommands::ResetNetworkDevice));
}

TEST_F(NvidiaSwitchResetControlTest, UnsupportedResetTypeIsRejected)
{
    // An unsupported reset type must be rejected before any MCTP traffic.
    EXPECT_CALL(mctpMock, sendRecvMsg).Times(0);

    auto ctrl = createControl("cx9_reject");
    EXPECT_TRUE(setPendingReset(
        "cx9_reject",
        "xyz.openbmc_project.Control.Reset.ResetTypes.GracefulRestart"));

    EXPECT_EQ(getProperty<std::string>(resetPath("cx9_reject"), resetIfaceName,
                                       "PendingReset"),
              noneValue);
}

} // namespace
