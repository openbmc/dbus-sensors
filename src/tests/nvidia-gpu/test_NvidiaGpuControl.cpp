/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuControl.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <MessagePackUnpackUtils.hpp>
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
    // Response: header(5) + command(1) + cc(1) + reasonCode(2) + dataSize(2)
    //           + 3*uint32_t(12) = 23 bytes total
    const uint16_t dataSize = sizeof(uint32_t) * 3;
    std::vector<uint8_t> buf(23, 0);
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

class NvidiaGpuControlTestBase : public DbusMockTestBase
{
  protected:
    // Create a powerCapInterface that GpuControl reads/writes
    static std::shared_ptr<sdbusplus::asio::dbus_interface>
        createPowerCapInterface(const std::string& name)
    {
        const std::string path =
            "/xyz/openbmc_project/control/power/" + name + "_iface";
        auto iface = objectServer->add_interface(
            path, "xyz.openbmc_project.Control.Power.Cap");
        iface->register_property("PowerCap",
                                 std::numeric_limits<uint32_t>::max());
        iface->register_property("PowerCapEnable", false);
        iface->register_property("MinPowerCapValue", uint32_t{0});
        iface->register_property("MaxPowerCapValue",
                                 std::numeric_limits<uint32_t>::max());
        iface->register_property("DefaultPowerCap",
                                 std::numeric_limits<uint32_t>::max(),
                                 sdbusplus::asio::PropertyPermission::readOnly);
        iface->initialize();
        return iface;
    }

    static std::shared_ptr<NvidiaGpuControl> createControl(
        const std::string& name = "GPU_CTRL",
        uint8_t eid = test_utils::defaultEid)
    {
        auto powerCapIface = createPowerCapInterface(name);
        const std::string invPath = "/xyz/openbmc_project/inventory/" + name;
        return std::make_shared<NvidiaGpuControl>(
            *objectServer, name, invPath, *mctpRequester, eid, powerCapIface);
    }
};

// ── Constructor ─────────────────────────────────────────────────────

TEST_F(NvidiaGpuControlTestBase, ConstructorDoesNotCrash)
{
    auto ctrl = createControl("ctrl_ctor");
    ASSERT_NE(ctrl, nullptr);
}

// ── Update (success path) ───────────────────────────────────────────

TEST_F(NvidiaGpuControlTestBase, UpdateSuccessSetsProperties)
{
    constexpr uint32_t enforcedMw = 300000; // 300W
    mock_mctp::setNextResponse(
        {}, buildPowerLimitsResponse(400000, 350000, enforcedMw));
    auto ctrl = createControl("ctrl_succ");
    ctrl->update();

    // Verify the D-Bus properties are set
    const std::string path =
        "/xyz/openbmc_project/control/power/ctrl_succ_iface";
    EXPECT_EQ(getProperty<uint32_t>(
                  path, "xyz.openbmc_project.Control.Power.Cap", "PowerCap"),
              enforcedMw / milliwattsPerWatt);
    EXPECT_TRUE(getProperty<bool>(path, "xyz.openbmc_project.Control.Power.Cap",
                                  "PowerCapEnable"));
}

TEST_F(NvidiaGpuControlTestBase, UpdateUnlimitedPowerCapDisablesFlag)
{
    // enforced = max uint32 → PowerCapEnable=false
    mock_mctp::setNextResponse(
        {},
        buildPowerLimitsResponse(0, 0, std::numeric_limits<uint32_t>::max()));
    auto ctrl = createControl("ctrl_unlim");
    ctrl->update();

    const std::string path =
        "/xyz/openbmc_project/control/power/ctrl_unlim_iface";
    EXPECT_FALSE(getProperty<bool>(
        path, "xyz.openbmc_project.Control.Power.Cap", "PowerCapEnable"));
}

TEST_F(NvidiaGpuControlTestBase, UpdateZeroEnforcedDisablesFlag)
{
    // enforced = 0 → PowerCapEnable=false
    mock_mctp::setNextResponse({}, buildPowerLimitsResponse(0, 0, 0));
    auto ctrl = createControl("ctrl_zero");
    ctrl->update();

    const std::string path =
        "/xyz/openbmc_project/control/power/ctrl_zero_iface";
    EXPECT_FALSE(getProperty<bool>(
        path, "xyz.openbmc_project.Control.Power.Cap", "PowerCapEnable"));
}

// ── Update (sends request) ──────────────────────────────────────────

TEST_F(NvidiaGpuControlTestBase, UpdateSendsRequest)
{
    mock_mctp::setNextResponse({}, {});
    auto ctrl = createControl("ctrl_sends");
    ctrl->update();
    EXPECT_GE(mock_mctp::getRequestCount(), 1U);
}

TEST_F(NvidiaGpuControlTestBase, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    mock_mctp::setNextResponse({}, {});
    auto ctrl = createControl("ctrl_eid", testEid);
    ctrl->update();
    EXPECT_EQ(mock_mctp::getLastEid(), testEid);
}

TEST_F(NvidiaGpuControlTestBase, UpdateVerifiesRequestEncoding)
{
    mock_mctp::setNextResponse({}, {});
    auto ctrl = createControl("ctrl_enc");
    ctrl->update();

    const std::span<const uint8_t> req = mock_mctp::getLastRequest();
    ASSERT_FALSE(req.empty());

    UnpackBuffer unpack(req);
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

TEST_F(NvidiaGpuControlTestBase, UpdateMctpTransportErrorNoCrash)
{
    mock_mctp::setNextResponse(std::make_error_code(std::errc::timed_out), {});
    auto ctrl = createControl("ctrl_mctp_err");
    EXPECT_NO_THROW(ctrl->update());
}

TEST_F(NvidiaGpuControlTestBase, UpdateDecodeErrorNoCrash)
{
    mock_mctp::setNextResponse({}, buildErrorResponse());
    auto ctrl = createControl("ctrl_dec_err");
    EXPECT_NO_THROW(ctrl->update());
}

TEST_F(NvidiaGpuControlTestBase, UpdateEmptyBufferNoCrash)
{
    mock_mctp::setNextResponse({}, {});
    auto ctrl = createControl("ctrl_empty");
    EXPECT_NO_THROW(ctrl->update());
}

TEST_F(NvidiaGpuControlTestBase, UpdateTinyBufferNoCrash)
{
    mock_mctp::setNextResponse({}, {0x00, 0x01});
    auto ctrl = createControl("ctrl_tiny");
    EXPECT_NO_THROW(ctrl->update());
}

// ── Destructor ──────────────────────────────────────────────────────

TEST_F(NvidiaGpuControlTestBase, DestructorRemovesInterface)
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
    // After destruction, the property read must fail.
    EXPECT_THROW(getProperty<std::vector<Association>>(
                     assocPath, "xyz.openbmc_project.Association.Definitions",
                     "Associations"),
                 sdbusplus::exception_t);
}

} // namespace
