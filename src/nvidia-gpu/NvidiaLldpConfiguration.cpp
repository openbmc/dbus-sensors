/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaLldpConfiguration.hpp"

#include "MctpRequester.hpp"
#include "Utils.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace
{

constexpr const char* configurationInterfaceName =
    "xyz.openbmc_project.Network.LLDP.Configuration";

constexpr std::string_view modePrefix =
    "xyz.openbmc_project.Network.LLDP.Configuration.Mode.";

// The LLDP objects of a device are gathered under one path so that they are
// reachable without walking the inventory, which they are not part of.
constexpr const char* lldpPathPrefix = "/xyz/openbmc_project/network/lldp";

// Long enough to gather the writes of one client request, short enough that a
// client waiting on the result does not notice.
constexpr auto setModeDebounce = std::chrono::milliseconds(100);

std::string toModeString(gpu::LldpMode mode)
{
    switch (mode)
    {
        case gpu::LldpMode::Mandatory:
            return std::string(modePrefix) + "Mandatory";
        case gpu::LldpMode::All:
            return std::string(modePrefix) + "All";
        case gpu::LldpMode::Off:
        default:
            return std::string(modePrefix) + "Disabled";
    }
}

// Returns whether requested names a mode, and what it names. A string the
// interface does not define is a client error rather than a device one.
bool fromModeString(const std::string& requested, gpu::LldpMode& mode)
{
    if (!requested.starts_with(modePrefix))
    {
        return false;
    }

    const std::string_view name{requested};
    const std::string_view value = name.substr(modePrefix.size());

    if (value == "Disabled")
    {
        mode = gpu::LldpMode::Off;
        return true;
    }
    if (value == "Mandatory")
    {
        mode = gpu::LldpMode::Mandatory;
        return true;
    }
    if (value == "All")
    {
        mode = gpu::LldpMode::All;
        return true;
    }
    return false;
}

gpu::LldpMode modeFromByte(uint8_t modeData, uint8_t shift)
{
    switch ((modeData >> shift) & gpu::lldpModeMask)
    {
        case static_cast<uint8_t>(gpu::LldpMode::Mandatory):
            return gpu::LldpMode::Mandatory;
        case static_cast<uint8_t>(gpu::LldpMode::All):
            return gpu::LldpMode::All;
        default:
            // The remaining value is reserved. Reporting it as off says the
            // agent is doing nothing, which is the safest thing to claim about
            // a setting this interface cannot name.
            return gpu::LldpMode::Off;
    }
}

uint8_t byteWithMode(uint8_t modeData, uint8_t shift, gpu::LldpMode mode)
{
    const auto cleared =
        static_cast<uint8_t>(modeData & ~(gpu::lldpModeMask << shift));
    return static_cast<uint8_t>(
        cleared | (static_cast<uint8_t>(mode) << shift));
}

} // namespace

NvidiaLldpConfiguration::NvidiaLldpConfiguration(
    sdbusplus::asio::object_server& objectServer,
    mctp::MctpRequester& mctpRequester, const std::string& deviceName,
    const sdbusplus::object_path& adapterPath, uint8_t eid,
    boost::asio::io_context& io) :
    objectServer(objectServer), mctpRequester(mctpRequester),
    deviceName(deviceName), adapterPath(adapterPath),
    objectPath(sdbusplus::object_path(lldpPathPrefix) / deviceName), eid(eid),
    setModeTimer(io)
{}

NvidiaLldpConfiguration::~NvidiaLldpConfiguration()
{
    if (configurationInterface)
    {
        objectServer.remove_interface(configurationInterface);
    }
    if (associationInterface)
    {
        objectServer.remove_interface(associationInterface);
    }
}

void NvidiaLldpConfiguration::update()
{
    sendGetModeRequest();
}

void NvidiaLldpConfiguration::sendGetModeRequest()
{
    const int rc = gpu::encodeGetDeviceModeSettingsV2Request(
        0, gpu::DeviceMode::LLDP, getRequest);

    if (rc != 0)
    {
        lg2::error(
            "Error reading the LLDP mode of {NAME}: encode failed, rc={RC}, eid={EID}",
            "NAME", deviceName, "RC", rc, "EID", eid);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, getRequest,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaLldpConfiguration> self = weak.lock();
            if (self == nullptr)
            {
                return;
            }
            self->handleGetModeResponse(ec, buffer);
        });
}

void NvidiaLldpConfiguration::handleGetModeResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    setInFlight = false;

    if (ec)
    {
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint8_t modeData = 0;

    const int rc = gpu::decodeGetDeviceModeSettingsV2Response(
        buffer, cc, reasonCode, modeData);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        // A device that does not hold this mode answers every poll the same
        // way, so say so once and leave the object unpublished.
        if (!published && !reportedUnsupported)
        {
            reportedUnsupported = true;
            lg2::info(
                "{NAME} reports no LLDP mode, so its LLDP is not configurable: rc={RC}, cc={CC}, eid={EID}",
                "NAME", deviceName, "RC", rc, "CC", static_cast<uint8_t>(cc),
                "EID", eid);
        }
        return;
    }

    if (!published)
    {
        publish(modeData);
        return;
    }

    if (configurationInterface == nullptr)
    {
        return;
    }

    deviceMode = modeData;

    const gpu::LldpMode transmit =
        modeFromByte(modeData, gpu::lldpTransmitModeShift);
    const gpu::LldpMode receive =
        modeFromByte(modeData, gpu::lldpReceiveModeShift);

    if (transmit != transmitMode)
    {
        transmitMode = transmit;
        configurationInterface->signal_property("TransmitMode");
    }
    if (receive != receiveMode)
    {
        receiveMode = receive;
        configurationInterface->signal_property("ReceiveMode");
    }
}

void NvidiaLldpConfiguration::publish(uint8_t modeData)
{
    deviceMode = modeData;
    transmitMode = modeFromByte(modeData, gpu::lldpTransmitModeShift);
    receiveMode = modeFromByte(modeData, gpu::lldpReceiveModeShift);

    configurationInterface =
        objectServer.add_interface(objectPath, configurationInterfaceName);

    // The interface is torn down in the destructor, so a callback cannot
    // outlive the object it reads and writes.
    configurationInterface->register_property<std::string>(
        "TransmitMode", toModeString(transmitMode),
        std::bind_front(&NvidiaLldpConfiguration::handleTransmitModeSet, this),
        [this](std::string&) { return toModeString(transmitMode); });

    configurationInterface->register_property<std::string>(
        "ReceiveMode", toModeString(receiveMode),
        std::bind_front(&NvidiaLldpConfiguration::handleReceiveModeSet, this),
        [this](std::string&) { return toModeString(receiveMode); });

    if (!configurationInterface->initialize())
    {
        lg2::error(
            "Error initializing LLDP configuration for {NAME}, eid={EID}",
            "NAME", deviceName, "EID", eid);
        return;
    }

    std::vector<Association> associations;
    associations.emplace_back("controlling", "controlled_by", adapterPath.str);

    associationInterface =
        objectServer.add_interface(objectPath, association::interface);
    associationInterface->register_property("Associations", associations);

    if (!associationInterface->initialize())
    {
        lg2::error(
            "Error initializing Association interface for the LLDP configuration of {NAME}, eid={EID}",
            "NAME", deviceName, "EID", eid);
    }

    published = true;
}

int NvidiaLldpConfiguration::handleTransmitModeSet(const std::string& requested,
                                                   std::string& /*current*/)
{
    gpu::LldpMode mode{};
    if (!fromModeString(requested, mode))
    {
        throw sdbusplus::xyz::openbmc_project::Common::Error::InvalidArgument();
    }

    // DCBX rides on LLDP and needs every field of a frame in both directions.
    // Leaving it enabled over a narrower mode would break it, so the write is
    // refused rather than silently breaking a protocol this interface does not
    // otherwise describe.
    if ((deviceMode & gpu::lldpDcbxEnabledBit) != 0 &&
        mode != gpu::LldpMode::All)
    {
        throw sdbusplus::xyz::openbmc_project::Common::Error::Unavailable();
    }

    pendingTransmitMode = mode;
    armSetModeTimer();

    // The exposed value follows the device, not the request, so a write that
    // the device refuses leaves nothing behind.
    return 1;
}

int NvidiaLldpConfiguration::handleReceiveModeSet(const std::string& requested,
                                                  std::string& /*current*/)
{
    gpu::LldpMode mode{};
    if (!fromModeString(requested, mode))
    {
        throw sdbusplus::xyz::openbmc_project::Common::Error::InvalidArgument();
    }

    if ((deviceMode & gpu::lldpDcbxEnabledBit) != 0 &&
        mode != gpu::LldpMode::All)
    {
        throw sdbusplus::xyz::openbmc_project::Common::Error::Unavailable();
    }

    pendingReceiveMode = mode;
    armSetModeTimer();

    return 1;
}

void NvidiaLldpConfiguration::armSetModeTimer()
{
    setModeTimer.expires_after(setModeDebounce);
    setModeTimer.async_wait(
        [weak{weak_from_this()}](const boost::system::error_code& ec) {
            if (ec)
            {
                // A later write restarted the timer; that write will dispatch.
                return;
            }
            std::shared_ptr<NvidiaLldpConfiguration> self = weak.lock();
            if (self == nullptr)
            {
                return;
            }
            self->applyPendingMode();
        });
}

void NvidiaLldpConfiguration::applyPendingMode()
{
    if (setInFlight)
    {
        // Wait for the device to answer the command already on its way rather
        // than sending a second one that would race it.
        armSetModeTimer();
        return;
    }

    if (!pendingTransmitMode && !pendingReceiveMode)
    {
        return;
    }

    uint8_t modeData = deviceMode;

    if (pendingTransmitMode)
    {
        modeData = byteWithMode(modeData, gpu::lldpTransmitModeShift,
                                *pendingTransmitMode);
    }
    if (pendingReceiveMode)
    {
        modeData = byteWithMode(modeData, gpu::lldpReceiveModeShift,
                                *pendingReceiveMode);
    }

    pendingTransmitMode.reset();
    pendingReceiveMode.reset();

    sendSetModeRequest(modeData);
}

void NvidiaLldpConfiguration::sendSetModeRequest(uint8_t modeData)
{
    const int rc = gpu::encodeSetDeviceModeSettingsV2Request(
        0, gpu::DeviceMode::LLDP, modeData, setRequest);

    if (rc != 0)
    {
        lg2::error(
            "Error setting the LLDP mode of {NAME}: encode failed, rc={RC}, eid={EID}",
            "NAME", deviceName, "RC", rc, "EID", eid);
        return;
    }

    setInFlight = true;

    mctpRequester.sendRecvMsg(
        eid, setRequest,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaLldpConfiguration> self = weak.lock();
            if (self == nullptr)
            {
                return;
            }
            self->handleSetModeResponse(ec, buffer);
        });
}

void NvidiaLldpConfiguration::handleSetModeResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    // The write is not finished until what the device now holds has been read
    // back: until then the byte a further write would build on is the one from
    // before this write, and building on it would undo what this write did.
    // Reading back is what clears the flag, so a write waiting behind this one
    // waits for that too.
    if (ec)
    {
        lg2::error("Error setting the LLDP mode of {NAME}: {ERROR}, eid={EID}",
                   "NAME", deviceName, "ERROR", ec.message(), "EID", eid);
        setInFlight = false;
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;

    const int rc =
        gpu::decodeSetDeviceModeSettingsV2Response(buffer, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error setting the LLDP mode of {NAME}: rc={RC}, cc={CC}, reasonCode={RESC}, eid={EID}",
            "NAME", deviceName, "RC", rc, "CC", static_cast<uint8_t>(cc),
            "RESC", reasonCode, "EID", eid);
    }

    // Read back either way: what the device holds now is the answer, whether
    // it took the write or refused it.
    sendGetModeRequest();
}
