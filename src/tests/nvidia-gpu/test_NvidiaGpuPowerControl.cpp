/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Inventory.hpp"
#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuControlErrors.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaGpuPowerControl.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/exception.hpp>

#include <chrono>
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

constexpr const char* powerCapIfaceName =
    "xyz.openbmc_project.Control.Power.Cap";

// Power-cap window the stub device reports through the inventory. The
// PowerCap setter accepts values inside it and rejects everything else.
constexpr uint32_t minCapWatts = 100;
constexpr uint32_t maxCapWatts = 500;

// Value seeded by a successful read before each error case, in milliwatts.
constexpr uint32_t seededOneshotMw = 300000;

// Comfortably longer than the control's 100 ms debounce window.
constexpr std::chrono::seconds debounceTimeout{5};

// Long enough that a second debounce window would have fired had the burst
// not been coalesced.
constexpr std::chrono::seconds quietWindow{1};

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

// Build a successful GET_INVENTORY_INFORMATION response whose payload starts
// with `leadingValue`. A 16-byte payload decodes cleanly for every property
// the Inventory asks for, and the power-limit properties consume the leading
// uint32.
std::vector<uint8_t> buildInventoryResponse(uint32_t leadingValue)
{
    constexpr uint16_t dataSize = 16;
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + dataSize, 0);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reasonCode
    pack.pack(dataSize);
    pack.pack(leadingValue);
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

// Command byte of a PLATFORM_ENVIRONMENTAL request; it directly follows the
// message header.
uint8_t requestCommand(const std::vector<uint8_t>& request)
{
    return request.size() > ocp::accelerator_management::messageHeaderSize
               ? request[ocp::accelerator_management::messageHeaderSize]
               : 0;
}

// Property id of a GetInventoryInformation request; it is the last byte.
uint8_t inventoryPropertyId(const std::vector<uint8_t>& request)
{
    return request.empty() ? 0 : request.back();
}

// Payload of a SetPowerLimits request, after the common header:
//   powerLimitId(4) + action(1) + persistence(1) + milliwatts(4)
struct SetPowerLimitsRequest
{
    uint8_t action{};
    uint8_t persistence{};
    uint32_t milliwatts{};
};

SetPowerLimitsRequest decodeSetPowerLimits(const std::vector<uint8_t>& request)
{
    UnpackBuffer unpack(request);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId,
                  msgType),
              0);

    uint8_t command = 0;
    uint8_t dataSize = 0;
    uint32_t powerLimitId = 0;
    SetPowerLimitsRequest decoded;
    unpack.unpack(command);
    unpack.unpack(dataSize);
    unpack.unpack(powerLimitId);
    unpack.unpack(decoded.action);
    unpack.unpack(decoded.persistence);
    unpack.unpack(decoded.milliwatts);
    EXPECT_EQ(unpack.getError(), 0);
    return decoded;
}

class NvidiaGpuPowerControlTest : public MctpMockTestBase
{
  protected:
    // The control registers PowerCap/PowerCapEnable itself and initializes
    // the interface, so mirror GpuDevice: register only the static limits.
    // The interface is kept on the fixture because the two setters are
    // private to the control, so the tests drive them through D-Bus.
    void makePowerCapInterface(const std::string& name)
    {
        powerCapInterface =
            objects().add_interface(powerCapPath(name), powerCapIfaceName);
        powerCapInterface->register_property("MinPowerCapValue", uint32_t{0});
        powerCapInterface->register_property(
            "MaxPowerCapValue", std::numeric_limits<uint32_t>::max());
        powerCapInterface->register_property(
            "DefaultPowerCap", std::numeric_limits<uint32_t>::max());
    }

    std::shared_ptr<NvidiaGpuPowerControl> createControl(
        const std::string& name = "GPU_CTRL",
        uint8_t eid = test_utils::defaultEid)
    {
        makePowerCapInterface(name);
        return std::make_shared<NvidiaGpuPowerControl>(
            objects(), name, requester(), eid, ioContext(), powerCapInterface,
            nullptr);
    }

    // GpuDevice hands one Power.Cap interface to both the Inventory and the
    // control; the Inventory is what supplies the [min, max] window the
    // PowerCap setter range-checks against. `fetchLimits` selects whether
    // that window has already been read from the device.
    std::shared_ptr<NvidiaGpuPowerControl> createControlWithInventory(
        const std::string& name, bool fetchLimits)
    {
        makePowerCapInterface(name);
        inventory = std::make_shared<Inventory>(
            bus(), objects(), name, requester(),
            gpu::DeviceIdentification::DEVICE_GPU, test_utils::defaultEid,
            ioContext(), powerCapInterface, nullptr);
        auto ctrl = std::make_shared<NvidiaGpuPowerControl>(
            objects(), name, requester(), test_utils::defaultEid, ioContext(),
            powerCapInterface, inventory);
        if (fetchLimits)
        {
            inventory->init();
        }
        return ctrl;
    }

    // Record every request and answer inventory queries with the stub
    // device's power-limit window. Anything else gets an empty buffer, which
    // the caller treats as an undecodable response: these tests assert on
    // what was sent, not on what came back.
    void installDeviceResponder()
    {
        ON_CALL(mctpMock, sendRecvMsg)
            .WillByDefault([this](uint8_t /*eid*/,
                                  std::span<const uint8_t> reqMsg,
                                  auto callback) {
                const std::vector<uint8_t>& request =
                    requests.emplace_back(reqMsg.begin(), reqMsg.end());
                if (requestCommand(request) !=
                    static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::
                                             GET_INVENTORY_INFORMATION))
                {
                    callback(std::error_code{}, std::span<const uint8_t>{});
                    return;
                }

                uint32_t value = 0;
                if (inventoryPropertyId(request) ==
                    static_cast<uint8_t>(
                        gpu::InventoryPropertyId::MIN_DEVICE_POWER_LIMIT))
                {
                    value = minCapWatts * milliwattsPerWatt;
                }
                else if (inventoryPropertyId(request) ==
                         static_cast<uint8_t>(
                             gpu::InventoryPropertyId::MAX_DEVICE_POWER_LIMIT))
                {
                    value = maxCapWatts * milliwattsPerWatt;
                }
                const std::vector<uint8_t> response =
                    buildInventoryResponse(value);
                callback(std::error_code{}, response);
            });
    }

    // The SetPowerLimits requests recorded by installDeviceResponder().
    std::vector<std::vector<uint8_t>> setPowerLimitRequests() const
    {
        std::vector<std::vector<uint8_t>> matches;
        for (const std::vector<uint8_t>& request : requests)
        {
            if (requestCommand(request) ==
                static_cast<uint8_t>(
                    gpu::PlatformEnvironmentalCommands::SET_POWER_LIMITS))
            {
                matches.push_back(request);
            }
        }
        return matches;
    }

    static std::string powerCapPath(const std::string& name)
    {
        return "/xyz/openbmc_project/control/power/" + name + "_iface";
    }

    // The D-Bus view of the cap; asserting it after a failed read is how the
    // error cases show the last good values survived.
    static void expectPowerCap(const std::string& name, uint32_t watts,
                               bool enabled)
    {
        EXPECT_EQ(getProperty<uint32_t>(powerCapPath(name), powerCapIfaceName,
                                        "PowerCap"),
                  watts);
        EXPECT_EQ(getProperty<bool>(powerCapPath(name), powerCapIfaceName,
                                    "PowerCapEnable"),
                  enabled);
    }

    std::shared_ptr<sdbusplus::asio::dbus_interface> powerCapInterface;
    std::shared_ptr<Inventory> inventory;
    std::vector<std::vector<uint8_t>> requests;
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
    expectPowerCap("ctrl_succ", oneshotMw / milliwattsPerWatt, true);
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

    EXPECT_FALSE(getProperty<bool>(powerCapPath("ctrl_unlim"),
                                   powerCapIfaceName, "PowerCapEnable"));
}

TEST_F(NvidiaGpuPowerControlTest, UpdateEnforcedMismatchDisablesFlag)
{
    // enforced != one-shot → PowerCapEnable=false
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildPowerLimitsResponse(400000, 350000, 300000)));
    auto ctrl = createControl("ctrl_mismatch");
    ctrl->update();

    EXPECT_FALSE(getProperty<bool>(powerCapPath("ctrl_mismatch"),
                                   powerCapIfaceName, "PowerCapEnable"));
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

// ── PowerCap set (range checks) ─────────────────────────────────────

TEST_F(NvidiaGpuPowerControlTest, PowerCapSetWithoutInventoryIsRejected)
{
    auto ctrl = createControl("ctrl_no_inv");

    // Without an Inventory the accepted window is unknown, so the setter has
    // to refuse rather than forward an unvalidated cap to the device.
    EXPECT_THROW(powerCapInterface->set_property("PowerCap", uint32_t{200}),
                 Unavailable);
}

TEST_F(NvidiaGpuPowerControlTest, PowerCapSetBeforeLimitsAreKnownIsRejected)
{
    // The Inventory exists but has not read the device's limits yet.
    auto ctrl = createControlWithInventory("ctrl_no_limits", false);

    EXPECT_THROW(powerCapInterface->set_property("PowerCap", uint32_t{200}),
                 Unavailable);
}

TEST_F(NvidiaGpuPowerControlTest, PowerCapSetOutsideDeviceWindowIsRejected)
{
    installDeviceResponder();
    auto ctrl = createControlWithInventory("ctrl_range", true);

    // InvalidArgument rather than Unavailable proves the window itself was
    // known and the value is what failed.
    EXPECT_THROW(powerCapInterface->set_property("PowerCap", minCapWatts - 1),
                 InvalidArgument);
    EXPECT_THROW(powerCapInterface->set_property("PowerCap", maxCapWatts + 1),
                 InvalidArgument);

    // A rejected cap must never reach the device, not even after the
    // debounce window a successful write would have used.
    EXPECT_FALSE(pumpIoUntil(
        [this] { return !setPowerLimitRequests().empty(); }, quietWindow));
}

// ── PowerCap set (debounce coalescing) ──────────────────────────────

TEST_F(NvidiaGpuPowerControlTest, CapAndEnableWritesCoalesceIntoOneRequest)
{
    installDeviceResponder();
    auto ctrl = createControlWithInventory("ctrl_debounce", true);

    // A PATCH of both properties arrives as two back-to-back writes. Each one
    // re-arms the debounce timer, so the burst has to produce a single
    // SetPowerLimits carrying the cap from the first write and the
    // NEW_LIMIT action implied by the second.
    ASSERT_NO_THROW(powerCapInterface->set_property("PowerCap", maxCapWatts));
    ASSERT_NO_THROW(powerCapInterface->set_property("PowerCapEnable", true));

    ASSERT_TRUE(pumpIoUntil([this] { return !setPowerLimitRequests().empty(); },
                            debounceTimeout));
    EXPECT_FALSE(pumpIoUntil(
        [this] { return setPowerLimitRequests().size() > 1; }, quietWindow));

    const std::vector<std::vector<uint8_t>> sets = setPowerLimitRequests();
    ASSERT_EQ(sets.size(), 1U);

    const SetPowerLimitsRequest decoded = decodeSetPowerLimits(sets.front());
    EXPECT_EQ(decoded.milliwatts, maxCapWatts * milliwattsPerWatt);
    EXPECT_EQ(decoded.action,
              static_cast<uint8_t>(gpu::SetPowerLimitsAction::NEW_LIMIT));
    EXPECT_EQ(decoded.persistence,
              static_cast<uint8_t>(gpu::SetPowerLimitsPersistence::ONE_SHOT));
}

// ── Error handling ──────────────────────────────────────────────────
//
// Each failure path seeds a good reading first, so the assertion is that the
// failure left the previously decoded values in place.

TEST_F(NvidiaGpuPowerControlTest, UpdateMctpTransportErrorKeepsPreviousValues)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {},
            buildPowerLimitsResponse(400000, seededOneshotMw, seededOneshotMw)))
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));
    auto ctrl = createControl("ctrl_mctp_err");
    ctrl->update();
    ctrl->update();

    expectPowerCap("ctrl_mctp_err", seededOneshotMw / milliwattsPerWatt, true);
}

TEST_F(NvidiaGpuPowerControlTest, UpdateDecodeErrorKeepsPreviousValues)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {},
            buildPowerLimitsResponse(400000, seededOneshotMw, seededOneshotMw)))
        .WillOnce(mock_mctp::respondWith({}, buildErrorResponse()));
    auto ctrl = createControl("ctrl_dec_err");
    ctrl->update();
    ctrl->update();

    expectPowerCap("ctrl_dec_err", seededOneshotMw / milliwattsPerWatt, true);
}

TEST_F(NvidiaGpuPowerControlTest, UpdateEmptyBufferKeepsPreviousValues)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {},
            buildPowerLimitsResponse(400000, seededOneshotMw, seededOneshotMw)))
        .WillOnce(mock_mctp::respondWith({}, {}));
    auto ctrl = createControl("ctrl_empty");
    ctrl->update();
    ctrl->update();

    expectPowerCap("ctrl_empty", seededOneshotMw / milliwattsPerWatt, true);
}

TEST_F(NvidiaGpuPowerControlTest, UpdateTinyBufferKeepsPreviousValues)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {},
            buildPowerLimitsResponse(400000, seededOneshotMw, seededOneshotMw)))
        .WillOnce(mock_mctp::respondWith({}, {0x00, 0x01}));
    auto ctrl = createControl("ctrl_tiny");
    ctrl->update();
    ctrl->update();

    expectPowerCap("ctrl_tiny", seededOneshotMw / milliwattsPerWatt, true);
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
