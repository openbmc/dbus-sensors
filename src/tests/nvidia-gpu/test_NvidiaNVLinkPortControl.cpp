/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaNVLinkPortControl.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

// NOLINTNEXTLINE(misc-include-cleaner): sd_bus_error lives here
#include <systemd/sd-bus.h>

#include <boost/system/error_code.hpp>
#include <sdbusplus/exception.hpp>
#include <sdbusplus/message.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultPortCount = 2;

constexpr const char* linkStateInterfaceName =
    "xyz.openbmc_project.Control.Port.LinkState";

constexpr const char* linkStateUnknown =
    "xyz.openbmc_project.Inventory.Connector.Port.LinkState.Unknown";
constexpr const char* linkStateEnabled =
    "xyz.openbmc_project.Inventory.Connector.Port.LinkState.Enabled";
constexpr const char* linkStateDisabled =
    "xyz.openbmc_project.Inventory.Connector.Port.LinkState.Disabled";

// Upper bound on how long a set may take to reach the device after its
// debounce window.
constexpr std::chrono::seconds debounceTimeout{5};

// Several debounce windows, so a request that should never be sent is caught.
constexpr std::chrono::seconds quietWindow{1};

using PortMask = std::array<uint8_t, gpu::portDisableFutureMaskSize>;

// Marks the port with the given (1-based) port number as disabled.
void disablePort(PortMask& mask, size_t portNumber)
{
    mask[portNumber / 8] |= static_cast<uint8_t>(1U << (portNumber % 8));
}

// Get Port Disable Future (Nvidia MCTP VDM 0x45): the 256-bit port mask.
std::vector<uint8_t> buildPortDisableFutureResponse(const PortMask& mask)
{
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + mask.size());
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(
        static_cast<uint8_t>(gpu::NetworkPortCommands::GetPortDisableFuture));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(mask.size()));
    for (const uint8_t byte : mask)
    {
        pack.pack(byte);
    }
    return buf;
}

std::vector<uint8_t> buildErrorResponse(
    ocp::accelerator_management::CompletionCode cc,
    gpu::NetworkPortCommands command =
        gpu::NetworkPortCommands::GetPortDisableFuture)
{
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(static_cast<uint8_t>(command));
    pack.pack(static_cast<uint8_t>(cc));
    pack.pack(static_cast<uint16_t>(0)); // reason code
    return buf;
}

// Set Port Disable Future (Nvidia MCTP VDM 0x44): success carries no data.
std::vector<uint8_t> buildSetPortDisableFutureResponse()
{
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(
        static_cast<uint8_t>(gpu::NetworkPortCommands::SetPortDisableFuture));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(0)); // data size
    return buf;
}

// Command byte of a request; it directly follows the message header.
uint8_t requestCommand(const std::vector<uint8_t>& request)
{
    return request.size() > ocp::accelerator_management::messageHeaderSize
               ? request[ocp::accelerator_management::messageHeaderSize]
               : 0;
}

// The port mask carried by a Set Port Disable Future request, which follows
// the command and data size bytes.
PortMask requestMask(const std::vector<uint8_t>& request)
{
    PortMask mask{};
    const size_t offset = ocp::accelerator_management::commonRequestSize;
    if (request.size() >= offset + mask.size())
    {
        std::ranges::copy_n(
            request.begin() + static_cast<std::ptrdiff_t>(offset),
            static_cast<std::ptrdiff_t>(mask.size()), mask.begin());
    }
    return mask;
}

class NvidiaNVLinkPortControlTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaNVLinkPortControl> createControl(
        const std::string& gpuName, uint8_t eid = test_utils::defaultEid,
        uint8_t numberOfPorts = defaultPortCount)
    {
        return std::make_shared<NvidiaNVLinkPortControl>(
            objects(), requester(), ioContext(), gpuName, eid, numberOfPorts);
    }

    static std::string controlPath(const std::string& gpuName,
                                   uint8_t portIndex)
    {
        return "/xyz/openbmc_project/control/port/" + gpuName + "/NVLink_" +
               std::to_string(portIndex);
    }

    static std::string requestedLinkState(const std::string& gpuName,
                                          uint8_t portIndex)
    {
        return getProperty<std::string>(controlPath(gpuName, portIndex),
                                        linkStateInterfaceName,
                                        "RequestedLinkState");
    }

    // Result of a D-Bus property write: the error name lets a rejected write
    // be told apart from a transport failure.
    struct SetResult
    {
        bool succeeded{false};
        std::string errorName;
    };

    // Writes RequestedLinkState on this connection's own object server. Async
    // because sd-bus rejects self-directed synchronous calls with ELOOP.
    static SetResult setRequestedLinkState(
        const std::string& gpuName, uint8_t portIndex, const std::string& value)
    {
        auto result = std::make_shared<std::optional<SetResult>>();

        bus()->async_method_call(
            [result](const boost::system::error_code& ec,
                     sdbusplus::message_t& reply) {
                SetResult outcome;
                outcome.succeeded = !ec;
                // NOLINTNEXTLINE(misc-include-cleaner): from sd-bus.h
                const sd_bus_error* error = reply.get_error();
                if (error != nullptr && error->name != nullptr)
                {
                    outcome.errorName = error->name;
                }
                *result = outcome;
            },
            bus()->get_unique_name(), controlPath(gpuName, portIndex),
            "org.freedesktop.DBus.Properties", "Set", linkStateInterfaceName,
            "RequestedLinkState", std::variant<std::string>(value));

        pumpIoUntil([result] { return result->has_value(); }, debounceTimeout);
        return result->value_or(SetResult{false, "timeout"});
    }

    // Stand in for the device: answer Get Port Disable Future with
    // deviceMask, and apply each accepted Set Port Disable Future to it, the
    // way the device keeps the mask across requests. Every request is
    // recorded.
    void installDeviceResponder()
    {
        ON_CALL(mctpMock, sendRecvMsg)
            .WillByDefault(
                [this](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                       auto callback) {
                    const std::vector<uint8_t>& request =
                        sentRequests.emplace_back(reqMsg.begin(), reqMsg.end());
                    std::vector<uint8_t> response;
                    if (requestCommand(request) ==
                        static_cast<uint8_t>(
                            gpu::NetworkPortCommands::SetPortDisableFuture))
                    {
                        if (holdSetResponses)
                        {
                            // Apply the mask now but answer only when the
                            // test releases it, as a slow device would.
                            deviceMask = requestMask(request);
                            heldSetCallback = std::move(callback);
                            return;
                        }
                        if (setTimesOut)
                        {
                            // The device takes the mask, but the answer is
                            // lost on the way back.
                            deviceMask = requestMask(request);
                            callback(std::make_error_code(std::errc::timed_out),
                                     std::span<const uint8_t>{});
                            return;
                        }
                        if (rejectSet)
                        {
                            response = buildErrorResponse(
                                ocp::accelerator_management::CompletionCode::
                                    ERR_INVALID_DATA,
                                gpu::NetworkPortCommands::SetPortDisableFuture);
                        }
                        else
                        {
                            deviceMask = requestMask(request);
                            response = buildSetPortDisableFutureResponse();
                        }
                    }
                    else
                    {
                        response = buildPortDisableFutureResponse(deviceMask);
                    }
                    callback(std::error_code{}, response);
                });
    }

    // The Set Port Disable Future requests recorded by installDeviceResponder.
    std::vector<std::vector<uint8_t>> setRequests() const
    {
        std::vector<std::vector<uint8_t>> matches;
        for (const std::vector<uint8_t>& request : sentRequests)
        {
            if (requestCommand(request) ==
                static_cast<uint8_t>(
                    gpu::NetworkPortCommands::SetPortDisableFuture))
            {
                matches.push_back(request);
            }
        }
        return matches;
    }

    // True once a Get Port Disable Future has been sent after a Set.
    bool readBackSent() const
    {
        const auto set = std::ranges::find_if(
            sentRequests, [](const std::vector<uint8_t>& request) {
                return requestCommand(request) ==
                       static_cast<uint8_t>(
                           gpu::NetworkPortCommands::SetPortDisableFuture);
            });
        return set != sentRequests.end() &&
               std::ranges::any_of(std::next(set), sentRequests.end(),
                                   [](const std::vector<uint8_t>& request) {
                                       return requestCommand(request) ==
                                              static_cast<uint8_t>(
                                                  gpu::NetworkPortCommands::
                                                      GetPortDisableFuture);
                                   });
    }

    // Creates the control and reads the device mask once, as the GPU device
    // does on its first poll, so writes have a mask to build on.
    std::shared_ptr<NvidiaNVLinkPortControl> createReadyControl(
        const std::string& gpuName)
    {
        installDeviceResponder();
        std::shared_ptr<NvidiaNVLinkPortControl> control =
            createControl(gpuName);
        control->update();
        return control;
    }

    PortMask deviceMask{};
    bool rejectSet{false};
    bool setTimesOut{false};
    bool holdSetResponses{false};
    std::move_only_function<void(const std::error_code&,
                                 std::span<const uint8_t>)>
        heldSetCallback;
    std::vector<std::vector<uint8_t>> sentRequests;
};

// Constructor — D-Bus object creation

TEST_F(NvidiaNVLinkPortControlTest, ConstructorPublishesUnknownPerPort)
{
    const std::string gpuName = "nvlink_ctrl_ctor";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl(gpuName);

    for (uint8_t i = 0; i < defaultPortCount; ++i)
    {
        EXPECT_EQ(requestedLinkState(gpuName, i), linkStateUnknown);
    }
}

TEST_F(NvidiaNVLinkPortControlTest, ConstructorAssociatesControlWithPort)
{
    const std::string gpuName = "nvlink_ctrl_assoc";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl(gpuName);

    using AssociationList =
        std::vector<std::tuple<std::string, std::string, std::string>>;
    const AssociationList associations = getProperty<AssociationList>(
        controlPath(gpuName, 1), "xyz.openbmc_project.Association.Definitions",
        "Associations");

    ASSERT_EQ(associations.size(), 1U);
    EXPECT_EQ(std::get<0>(associations[0]), "controlling");
    EXPECT_EQ(std::get<1>(associations[0]), "controlled_by");
    EXPECT_EQ(std::get<2>(associations[0]),
              "/xyz/openbmc_project/inventory/" + gpuName + "/NVLink_1");
}

TEST_F(NvidiaNVLinkPortControlTest, DestructorRemovesInterfaces)
{
    const std::string gpuName = "nvlink_ctrl_dtor";
    {
        const std::shared_ptr<NvidiaNVLinkPortControl> control =
            createControl(gpuName);
        EXPECT_NO_THROW(requestedLinkState(gpuName, 0));
    }
    drainPendingAsync();
    EXPECT_THROW(requestedLinkState(gpuName, 0), sdbusplus::exception_t);
}

// Update — the port mask maps onto RequestedLinkState

TEST_F(NvidiaNVLinkPortControlTest, UpdateMapsMaskBitsToRequestedLinkState)
{
    // NVLink_1 is port number 2.
    PortMask mask{};
    disablePort(mask, 2);
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildPortDisableFutureResponse(mask)));

    const std::string gpuName = "nvlink_ctrl_bits";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl(gpuName);
    control->update();

    EXPECT_EQ(requestedLinkState(gpuName, 0), linkStateEnabled);
    EXPECT_EQ(requestedLinkState(gpuName, 1), linkStateDisabled);
}

TEST_F(NvidiaNVLinkPortControlTest, UpdateIgnoresMaskBitZero)
{
    // Port numbers start at 1, so bit 0 never belongs to a port.
    PortMask mask{};
    mask[0] = 0x01;
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildPortDisableFutureResponse(mask)));

    const std::string gpuName = "nvlink_ctrl_bit0";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl(gpuName);
    control->update();

    EXPECT_EQ(requestedLinkState(gpuName, 0), linkStateEnabled);
    EXPECT_EQ(requestedLinkState(gpuName, 1), linkStateEnabled);
}

TEST_F(NvidiaNVLinkPortControlTest, UpdateReadsPortsBeyondFirstMaskByte)
{
    // NVLink_8 is port number 9, the second bit of the second mask byte.
    constexpr uint8_t portCount = 10;
    PortMask mask{};
    disablePort(mask, 9);
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(
            mock_mctp::respondWith({}, buildPortDisableFutureResponse(mask)));

    const std::string gpuName = "nvlink_ctrl_byte1";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl(gpuName, test_utils::defaultEid, portCount);
    control->update();

    for (uint8_t i = 0; i < portCount; ++i)
    {
        EXPECT_EQ(requestedLinkState(gpuName, i),
                  i == 8 ? linkStateDisabled : linkStateEnabled);
    }
}

TEST_F(NvidiaNVLinkPortControlTest, UpdateVerifiesRequestEncoding)
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

    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl("nvlink_ctrl_req_enc");
    control->update();

    ASSERT_FALSE(lastRequest.empty());

    UnpackBuffer unpack(lastRequest);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    const int rc = ocp::accelerator_management::unpackHeader(
        unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId, msgType);
    EXPECT_EQ(rc, 0);
    EXPECT_EQ(ocpMsgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(msgType, static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));

    uint8_t command = 0;
    unpack.unpack(command);
    EXPECT_EQ(command, static_cast<uint8_t>(
                           gpu::NetworkPortCommands::GetPortDisableFuture));

    uint8_t dataSize = 0;
    unpack.unpack(dataSize);
    EXPECT_EQ(dataSize, 0);
    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaNVLinkPortControlTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl("nvlink_ctrl_eid", testEid);
    control->update();
}

// Error handling — the requested state stays at its last known value

TEST_F(NvidiaNVLinkPortControlTest, UpdateErrorCompletionCodeKeepsUnknown)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {},
            buildErrorResponse(
                ocp::accelerator_management::CompletionCode::ERR_NOT_READY)));

    const std::string gpuName = "nvlink_ctrl_cc_err";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl(gpuName);
    control->update();

    EXPECT_EQ(requestedLinkState(gpuName, 0), linkStateUnknown);
}

TEST_F(NvidiaNVLinkPortControlTest, UpdateMctpTransportErrorKeepsUnknown)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::string gpuName = "nvlink_ctrl_mctp_err";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl(gpuName);
    EXPECT_NO_THROW(control->update());

    EXPECT_EQ(requestedLinkState(gpuName, 0), linkStateUnknown);
}

TEST_F(NvidiaNVLinkPortControlTest, UpdateEmptyBufferKeepsUnknown)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    const std::string gpuName = "nvlink_ctrl_empty";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl(gpuName);
    EXPECT_NO_THROW(control->update());

    EXPECT_EQ(requestedLinkState(gpuName, 0), linkStateUnknown);
}

// RequestedLinkState set — validation

TEST_F(NvidiaNVLinkPortControlTest, SetBeforeMaskIsKnownIsRejected)
{
    installDeviceResponder();
    const std::string gpuName = "nvlink_ctrl_set_early";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createControl(gpuName);

    // The other ports' bits come from the device mask, which has not been read
    // yet, so the write cannot be built.
    const SetResult result =
        setRequestedLinkState(gpuName, 0, linkStateDisabled);
    EXPECT_FALSE(result.succeeded);
    EXPECT_EQ(result.errorName, "xyz.openbmc_project.Common.Error.Unavailable");

    EXPECT_FALSE(
        pumpIoUntil([this] { return !setRequests().empty(); }, quietWindow));
}

TEST_F(NvidiaNVLinkPortControlTest, SetUnknownIsRejected)
{
    const std::string gpuName = "nvlink_ctrl_set_unknown";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createReadyControl(gpuName);

    const SetResult result =
        setRequestedLinkState(gpuName, 0, linkStateUnknown);
    EXPECT_FALSE(result.succeeded);
    EXPECT_EQ(result.errorName,
              "xyz.openbmc_project.Common.Error.InvalidArgument");

    EXPECT_FALSE(
        pumpIoUntil([this] { return !setRequests().empty(); }, quietWindow));
}

// RequestedLinkState set — the request sent to the device

TEST_F(NvidiaNVLinkPortControlTest, SetDisabledKeepsOtherPortsInMask)
{
    // NVLink_0 (port number 1) is already disabled on the device.
    disablePort(deviceMask, 1);
    const std::string gpuName = "nvlink_ctrl_set_disable";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createReadyControl(gpuName);

    ASSERT_TRUE(setRequestedLinkState(gpuName, 1, linkStateDisabled).succeeded);
    ASSERT_TRUE(pumpIoUntil([this] { return !setRequests().empty(); },
                            debounceTimeout));

    // The write adds NVLink_1 (port number 2) without dropping NVLink_0.
    PortMask expected{};
    disablePort(expected, 1);
    disablePort(expected, 2);
    EXPECT_EQ(requestMask(setRequests().front()), expected);
}

TEST_F(NvidiaNVLinkPortControlTest, SetEnabledClearsPortBit)
{
    disablePort(deviceMask, 1);
    disablePort(deviceMask, 2);
    const std::string gpuName = "nvlink_ctrl_set_enable";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createReadyControl(gpuName);

    ASSERT_TRUE(setRequestedLinkState(gpuName, 0, linkStateEnabled).succeeded);
    ASSERT_TRUE(pumpIoUntil([this] { return !setRequests().empty(); },
                            debounceTimeout));

    PortMask expected{};
    disablePort(expected, 2);
    EXPECT_EQ(requestMask(setRequests().front()), expected);
}

TEST_F(NvidiaNVLinkPortControlTest, WritesToSeveralPortsCoalesceIntoOneSet)
{
    const std::string gpuName = "nvlink_ctrl_set_coalesce";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createReadyControl(gpuName);

    // The mask covers every port, so writes that arrive together have to
    // become a single request carrying all of them.
    ASSERT_TRUE(setRequestedLinkState(gpuName, 0, linkStateDisabled).succeeded);
    ASSERT_TRUE(setRequestedLinkState(gpuName, 1, linkStateDisabled).succeeded);

    ASSERT_TRUE(pumpIoUntil([this] { return !setRequests().empty(); },
                            debounceTimeout));
    EXPECT_FALSE(
        pumpIoUntil([this] { return setRequests().size() > 1; }, quietWindow));

    PortMask expected{};
    disablePort(expected, 1);
    disablePort(expected, 2);
    EXPECT_EQ(requestMask(setRequests().front()), expected);
}

TEST_F(NvidiaNVLinkPortControlTest, SetMatchingDeviceStateSendsNothing)
{
    disablePort(deviceMask, 1);
    const std::string gpuName = "nvlink_ctrl_set_same";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createReadyControl(gpuName);

    ASSERT_TRUE(setRequestedLinkState(gpuName, 0, linkStateDisabled).succeeded);

    EXPECT_FALSE(
        pumpIoUntil([this] { return !setRequests().empty(); }, quietWindow));
}

// RequestedLinkState set — the value only follows the device

TEST_F(NvidiaNVLinkPortControlTest, AcceptedSetIsReadBackFromDevice)
{
    const std::string gpuName = "nvlink_ctrl_set_readback";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createReadyControl(gpuName);
    ASSERT_EQ(requestedLinkState(gpuName, 1), linkStateEnabled);

    ASSERT_TRUE(setRequestedLinkState(gpuName, 1, linkStateDisabled).succeeded);

    // No poll runs here, so only the read-back after the accepted set can
    // move the value.
    ASSERT_TRUE(
        pumpIoUntil([this] { return readBackSent(); }, debounceTimeout));
    EXPECT_EQ(requestedLinkState(gpuName, 1), linkStateDisabled);
    EXPECT_EQ(requestedLinkState(gpuName, 0), linkStateEnabled);
}

TEST_F(NvidiaNVLinkPortControlTest, RejectedSetKeepsRequestedLinkState)
{
    rejectSet = true;
    const std::string gpuName = "nvlink_ctrl_set_rejected";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createReadyControl(gpuName);

    ASSERT_TRUE(setRequestedLinkState(gpuName, 1, linkStateDisabled).succeeded);
    ASSERT_TRUE(pumpIoUntil([this] { return !setRequests().empty(); },
                            debounceTimeout));

    // The device refused the mask: there is nothing to read back, and the
    // value still shows what the device last reported.
    EXPECT_FALSE(pumpIoUntil([this] { return readBackSent(); }, quietWindow));
    EXPECT_EQ(requestedLinkState(gpuName, 1), linkStateEnabled);
}

TEST_F(NvidiaNVLinkPortControlTest, TimedOutSetIsReadBackFromDevice)
{
    setTimesOut = true;
    const std::string gpuName = "nvlink_ctrl_set_timeout";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createReadyControl(gpuName);

    ASSERT_TRUE(setRequestedLinkState(gpuName, 1, linkStateDisabled).succeeded);

    // The answer was lost but the device may have taken the mask, so it is
    // read back instead of being assumed unchanged.
    ASSERT_TRUE(
        pumpIoUntil([this] { return readBackSent(); }, debounceTimeout));
    EXPECT_EQ(requestedLinkState(gpuName, 1), linkStateDisabled);
}

TEST_F(NvidiaNVLinkPortControlTest, WriteWaitsForInflightSetToBeReadBack)
{
    holdSetResponses = true;
    const std::string gpuName = "nvlink_ctrl_set_queued";
    const std::shared_ptr<NvidiaNVLinkPortControl> control =
        createReadyControl(gpuName);

    ASSERT_TRUE(setRequestedLinkState(gpuName, 0, linkStateDisabled).succeeded);
    ASSERT_TRUE(pumpIoUntil([this] { return !setRequests().empty(); },
                            debounceTimeout));

    // A second write while the first Set is unanswered must not be built on
    // the mask from before that Set.
    ASSERT_TRUE(setRequestedLinkState(gpuName, 1, linkStateDisabled).succeeded);
    EXPECT_FALSE(
        pumpIoUntil([this] { return setRequests().size() > 1; }, quietWindow));

    holdSetResponses = false;
    ASSERT_TRUE(heldSetCallback);
    const std::vector<uint8_t> response = buildSetPortDisableFutureResponse();
    heldSetCallback(std::error_code{}, response);

    ASSERT_TRUE(pumpIoUntil([this] { return setRequests().size() > 1; },
                            debounceTimeout));
    PortMask expected{};
    disablePort(expected, 1);
    disablePort(expected, 2);
    EXPECT_EQ(requestMask(setRequests().back()), expected);
}

} // namespace
