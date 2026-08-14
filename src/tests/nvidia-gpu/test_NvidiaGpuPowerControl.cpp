/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaGpuPowerControl.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/exception.hpp>

#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <tuple>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr uint32_t milliwattsPerWatt = 1000;

using Association = std::tuple<std::string, std::string, std::string>;

// Build a successful GET_POWER_LIMITS response:
//   CommonResponse header + 3 x uint32_t (persistent, oneshot, enforced)
std::vector<uint8_t> buildPowerLimitsResponse(
    uint32_t persistentMw, uint32_t oneshotMw, uint32_t enforcedMw)
{
    const uint16_t dataSize = sizeof(uint32_t) * 3;
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + dataSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_POWER_LIMITS));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reasonCode
    pack.pack(dataSize);
    pack.pack(persistentMw);
    pack.pack(oneshotMw);
    pack.pack(enforcedMw);
    EXPECT_EQ(pack.getError(), 0);
    return buf;
}

std::vector<uint8_t> buildErrorResponse()
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_POWER_LIMITS,
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::ERROR),
        0x1234);
}

class NvidiaGpuPowerControlTest : public MctpMockTestBase
{
  protected:
    // The control registers PowerCap/PowerCapEnable itself and initializes
    // the interface, so mirror GpuDevice: register only the static limits.
    static std::shared_ptr<sdbusplus::asio::dbus_interface>
        createPowerCapInterface(const std::string& name)
    {
        const std::string path =
            "/xyz/openbmc_project/control/power/" + name + "_iface";
        auto iface = objects().add_interface(
            path, "xyz.openbmc_project.Control.Power.Cap");
        iface->register_property("MinPowerCapValue", uint32_t{0});
        iface->register_property("MaxPowerCapValue",
                                 std::numeric_limits<uint32_t>::max());
        iface->register_property("DefaultPowerCap",
                                 std::numeric_limits<uint32_t>::max());
        return iface;
    }

    static std::shared_ptr<NvidiaGpuPowerControl> createControl(
        const std::string& name = "GPU_CTRL",
        uint8_t eid = test_utils::defaultEid)
    {
        auto powerCapIface = createPowerCapInterface(name);
        return std::make_shared<NvidiaGpuPowerControl>(
            objects(), name, requester(), eid, ioContext(), powerCapIface,
            nullptr);
    }
};

// ── Constructor ─────────────────────────────────────────────────────

TEST_F(NvidiaGpuPowerControlTest, ConstructorDoesNotCrash)
{
    auto ctrl = createControl("ctrl_ctor");
    ASSERT_NE(ctrl, nullptr);
}

// ── Update (success path) ───────────────────────────────────────────

TEST_F(NvidiaGpuPowerControlTest, UpdateSuccessSetsProperties)
{
    // PowerCap mirrors the one-shot limit; the flag is enabled only when
    // the one-shot limit is the one being enforced.
    constexpr uint32_t oneshotMw = 300000; // 300W
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildPowerLimitsResponse(400000, oneshotMw, oneshotMw)));
    auto ctrl = createControl("ctrl_succ");
    ctrl->update();

    // Verify the D-Bus properties are set
    const std::string path =
        "/xyz/openbmc_project/control/power/ctrl_succ_iface";
    EXPECT_EQ(getProperty<uint32_t>(
                  path, "xyz.openbmc_project.Control.Power.Cap", "PowerCap"),
              oneshotMw / milliwattsPerWatt);
    EXPECT_TRUE(getProperty<bool>(path, "xyz.openbmc_project.Control.Power.Cap",
                                  "PowerCapEnable"));
}

TEST_F(NvidiaGpuPowerControlTest, UpdateUnlimitedPowerCapDisablesFlag)
{
    // enforced = max uint32 → PowerCapEnable=false
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildPowerLimitsResponse(
                    0, 0, std::numeric_limits<uint32_t>::max())));
    auto ctrl = createControl("ctrl_unlim");
    ctrl->update();

    const std::string path =
        "/xyz/openbmc_project/control/power/ctrl_unlim_iface";
    EXPECT_FALSE(getProperty<bool>(
        path, "xyz.openbmc_project.Control.Power.Cap", "PowerCapEnable"));
}

TEST_F(NvidiaGpuPowerControlTest, UpdateEnforcedMismatchDisablesFlag)
{
    // enforced != one-shot → PowerCapEnable=false
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildPowerLimitsResponse(400000, 350000, 300000)));
    auto ctrl = createControl("ctrl_mismatch");
    ctrl->update();

    const std::string path =
        "/xyz/openbmc_project/control/power/ctrl_mismatch_iface";
    EXPECT_FALSE(getProperty<bool>(
        path, "xyz.openbmc_project.Control.Power.Cap", "PowerCapEnable"));
}

// ── Update (sends request) ──────────────────────────────────────────

TEST_F(NvidiaGpuPowerControlTest, UpdateSendsRequest)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));
    auto ctrl = createControl("ctrl_sends");
    ctrl->update();
}

TEST_F(NvidiaGpuPowerControlTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, {}));
    auto ctrl = createControl("ctrl_eid", testEid);
    ctrl->update();
}

TEST_F(NvidiaGpuPowerControlTest, UpdateVerifiesRequestEncoding)
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

    auto ctrl = createControl("ctrl_enc");
    ctrl->update();

    ASSERT_FALSE(lastRequest.empty());

    UnpackBuffer unpack(lastRequest);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    const int rc = ocp::accelerator_management::unpackHeader(
        unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId, msgType);
    EXPECT_EQ(rc, 0);
    EXPECT_EQ(ocpMsgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(msgType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    uint8_t command = 0;
    unpack.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::GET_POWER_LIMITS));
    EXPECT_EQ(unpack.getError(), 0);
}

// ── Error handling ──────────────────────────────────────────────────

TEST_F(NvidiaGpuPowerControlTest, UpdateMctpTransportErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));
    auto ctrl = createControl("ctrl_mctp_err");
    EXPECT_NO_THROW(ctrl->update());
}

TEST_F(NvidiaGpuPowerControlTest, UpdateDecodeErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildErrorResponse()));
    auto ctrl = createControl("ctrl_dec_err");
    EXPECT_NO_THROW(ctrl->update());
}

TEST_F(NvidiaGpuPowerControlTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));
    auto ctrl = createControl("ctrl_empty");
    EXPECT_NO_THROW(ctrl->update());
}

TEST_F(NvidiaGpuPowerControlTest, UpdateTinyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, {0x00, 0x01}));
    auto ctrl = createControl("ctrl_tiny");
    EXPECT_NO_THROW(ctrl->update());
}

// ── Destructor ──────────────────────────────────────────────────────

TEST_F(NvidiaGpuPowerControlTest, DestructorRemovesInterface)
{
    const std::string name = "ctrl_dtor";
    const std::string assocPath = "/xyz/openbmc_project/control/power/" + name;
    {
        auto ctrl = createControl(name);
        ASSERT_NE(ctrl, nullptr);
        // While alive, the association interface is reachable.
        EXPECT_NO_THROW(getProperty<std::vector<Association>>(
            assocPath, "xyz.openbmc_project.Association.Definitions",
            "Associations"));
    }
    drainPendingAsync();
    // After destruction, the property read must fail.
    EXPECT_THROW(getProperty<std::vector<Association>>(
                     assocPath, "xyz.openbmc_project.Association.Definitions",
                     "Associations"),
                 sdbusplus::exception_t);
}

} // namespace
