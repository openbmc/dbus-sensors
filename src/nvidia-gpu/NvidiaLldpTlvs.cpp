/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaLldpTlvs.hpp"

#include "LldpFrame.hpp"
#include "MctpRequester.hpp"
#include "Utils.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

namespace
{

constexpr const char* tlvsInterfaceName =
    "xyz.openbmc_project.Network.LLDP.TLVs";

constexpr const char* directionPrefix =
    "xyz.openbmc_project.Network.LLDP.TLVs.Direction.";
constexpr const char* subtypePrefix =
    "xyz.openbmc_project.Network.LLDP.TLVs.IdSubtype.";
constexpr const char* capabilityPrefix =
    "xyz.openbmc_project.Network.LLDP.TLVs.SystemCapability.";

// The LLDP objects of a device are gathered under one path so that they are
// reachable without walking the inventory, which they are not part of.
constexpr const char* lldpPathPrefix = "/xyz/openbmc_project/network/lldp";

// What a management VLAN identifier holds when there is none to report.
constexpr uint64_t noManagementVlanId = std::numeric_limits<uint64_t>::max();

// The device reports the frame as it travelled: two addresses and the type
// that says what follows, and only then the unit itself.
constexpr size_t ethernetHeaderSize = 14;
constexpr size_t ethernetTypeOffset = 12;
constexpr uint8_t lldpEthernetTypeHigh = 0x88;
constexpr uint8_t lldpEthernetTypeLow = 0xCC;

// The address every LLDP frame is sent to, which is what a header starts with.
constexpr std::array<uint8_t, 6> lldpDestination{0x01, 0x80, 0xC2,
                                                 0x00, 0x00, 0x0E};

// Returns the unit inside a frame, which is the frame itself if it arrived
// without a header. Devices differ on which they report, so which arrived is
// worked out rather than assumed. Both the address a frame is sent to and the
// type that says what follows are checked: either alone can occur inside a
// unit by chance, since a chassis identifier can hold anything, but a unit
// carrying both, at both of those offsets, cannot be constructed by accident.
std::span<const uint8_t> unitOf(std::span<const uint8_t> frame)
{
    if (frame.size() > ethernetHeaderSize &&
        std::equal(lldpDestination.begin(), lldpDestination.end(),
                   frame.begin()) &&
        frame[ethernetTypeOffset] == lldpEthernetTypeHigh &&
        frame[ethernetTypeOffset + 1] == lldpEthernetTypeLow)
    {
        return frame.subspan(ethernetHeaderSize);
    }
    return frame;
}

std::string toDirectionString(gpu::LldpPacketType direction)
{
    return std::string(directionPrefix) +
           (direction == gpu::LldpPacketType::Received ? "Received"
                                                       : "Transmitted");
}

std::string toSubtypeString(lldp::IdSubtype subtype)
{
    const std::string prefix{subtypePrefix};
    switch (subtype)
    {
        case lldp::IdSubtype::AgentId:
            return prefix + "AgentId";
        case lldp::IdSubtype::ChassisComp:
            return prefix + "ChassisComp";
        case lldp::IdSubtype::IfAlias:
            return prefix + "IfAlias";
        case lldp::IdSubtype::IfName:
            return prefix + "IfName";
        case lldp::IdSubtype::LocalAssign:
            return prefix + "LocalAssign";
        case lldp::IdSubtype::MacAddr:
            return prefix + "MacAddr";
        case lldp::IdSubtype::NetworkAddr:
            return prefix + "NetworkAddr";
        case lldp::IdSubtype::PortComp:
            return prefix + "PortComp";
        case lldp::IdSubtype::NotTransmitted:
        default:
            return prefix + "NotTransmitted";
    }
}

std::string toCapabilityString(lldp::SystemCapability capability)
{
    const std::string prefix{capabilityPrefix};
    switch (capability)
    {
        case lldp::SystemCapability::Bridge:
            return prefix + "Bridge";
        case lldp::SystemCapability::DOCSISCableDevice:
            return prefix + "DOCSISCableDevice";
        case lldp::SystemCapability::Other:
            return prefix + "Other";
        case lldp::SystemCapability::Repeater:
            return prefix + "Repeater";
        case lldp::SystemCapability::Router:
            return prefix + "Router";
        case lldp::SystemCapability::Station:
            return prefix + "Station";
        case lldp::SystemCapability::Telephone:
            return prefix + "Telephone";
        case lldp::SystemCapability::WLANAccessPoint:
            return prefix + "WLANAccessPoint";
        case lldp::SystemCapability::None:
        default:
            return prefix + "None";
    }
}

std::vector<std::string> toCapabilityStrings(
    const std::vector<lldp::SystemCapability>& capabilities)
{
    std::vector<std::string> names;
    names.reserve(capabilities.size());
    for (const lldp::SystemCapability capability : capabilities)
    {
        names.push_back(toCapabilityString(capability));
    }
    return names;
}

} // namespace

NvidiaLldpTlvs::NvidiaLldpTlvs(
    sdbusplus::asio::object_server& objectServer,
    mctp::MctpRequester& mctpRequester, const std::string& deviceName,
    const std::string& portName, const sdbusplus::object_path& portPath,
    uint8_t eid, uint16_t portNumber, gpu::LldpPacketType direction) :
    objectServer(objectServer), mctpRequester(mctpRequester),
    objectPath(sdbusplus::object_path(lldpPathPrefix) / deviceName / portName /
               (direction == gpu::LldpPacketType::Received ? "received"
                                                           : "transmitted")),
    eid(eid)
{
    const int rc =
        gpu::encodeGetLldpPacketRequest(0, portNumber, direction, request);
    if (rc != 0)
    {
        lg2::error(
            "Error encoding the LLDP frame request for port {PN} of eid {EID}, rc={RC}",
            "PN", portNumber, "EID", eid, "RC", rc);
    }
    else
    {
        requestEncoded = true;
    }

    tlvsInterface = objectServer.add_interface(objectPath, tlvsInterfaceName);

    tlvsInterface->register_property("ChassisId", reported.chassisId);
    tlvsInterface->register_property(
        "ChassisIdSubtype", toSubtypeString(reported.chassisIdSubtype));
    tlvsInterface->register_property("Direction", toDirectionString(direction));
    tlvsInterface->register_property("ManagementAddressIPv4",
                                     reported.managementAddressIPv4);
    tlvsInterface->register_property("ManagementAddressIPv6",
                                     reported.managementAddressIPv6);
    tlvsInterface->register_property("ManagementAddressMAC",
                                     reported.managementAddressMAC);
    // The interface carries a management VLAN, but nothing in a frame says
    // what it is: the identifier IEEE 802.1 puts there is the VLAN the port
    // itself is on, which is a different thing and is not reported in its
    // place. The property is published at the value that means it is absent.
    tlvsInterface->register_property("ManagementVlanId", noManagementVlanId);
    tlvsInterface->register_property("PortId", reported.portId);
    tlvsInterface->register_property("PortIdSubtype",
                                     toSubtypeString(reported.portIdSubtype));
    tlvsInterface->register_property(
        "SystemCapabilities", toCapabilityStrings(reported.systemCapabilities));
    tlvsInterface->register_property("SystemDescription",
                                     reported.systemDescription);
    tlvsInterface->register_property("SystemName", reported.systemName);

    if (!tlvsInterface->initialize())
    {
        lg2::error(
            "Error initializing the LLDP frame of port {PN} of eid {EID}", "PN",
            portNumber, "EID", eid);
    }

    std::vector<Association> associations;
    associations.emplace_back("monitoring", "monitored_by", portPath.str);

    associationInterface =
        objectServer.add_interface(objectPath, association::interface);
    associationInterface->register_property("Associations", associations);

    if (!associationInterface->initialize())
    {
        lg2::error(
            "Error initializing Association interface for the LLDP frame of port {PN} of eid {EID}",
            "PN", portNumber, "EID", eid);
    }
}

NvidiaLldpTlvs::~NvidiaLldpTlvs()
{
    if (tlvsInterface)
    {
        objectServer.remove_interface(tlvsInterface);
    }
    if (associationInterface)
    {
        objectServer.remove_interface(associationInterface);
    }
}

void NvidiaLldpTlvs::update()
{
    if (!requestEncoded)
    {
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaLldpTlvs> self = weak.lock();
            if (self == nullptr)
            {
                return;
            }
            self->processResponse(ec, buffer);
        });
}

void NvidiaLldpTlvs::processResponse(const std::error_code& ec,
                                     std::span<const uint8_t> buffer)
{
    if (ec)
    {
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    std::vector<uint8_t> packet;

    const int rc =
        gpu::decodeGetLldpPacketResponse(buffer, cc, reasonCode, packet);

    if (rc != 0)
    {
        return;
    }

    // Everything that leaves a port with nothing to report ends here: the
    // device holding no frame, a frame it could not make sense of, or one
    // this decoder could not. None of them is a fault worth logging every
    // poll, and all of them mean the same thing to a client.
    lldp::Frame frame;
    if (cc == ocp::accelerator_management::CompletionCode::SUCCESS &&
        !packet.empty())
    {
        if (!lldp::parse(unitOf(packet), frame))
        {
            frame = lldp::Frame{};
        }
    }
    else if (cc != ocp::accelerator_management::CompletionCode::SUCCESS &&
             cc != ocp::accelerator_management::CompletionCode::ERR_NOT_READY)
    {
        return;
    }

    publish(frame);
}

void NvidiaLldpTlvs::publish(const lldp::Frame& frame)
{
    if (frame.chassisId != reported.chassisId)
    {
        reported.chassisId = frame.chassisId;
        tlvsInterface->set_property("ChassisId", reported.chassisId);
    }
    if (frame.chassisIdSubtype != reported.chassisIdSubtype)
    {
        reported.chassisIdSubtype = frame.chassisIdSubtype;
        tlvsInterface->set_property("ChassisIdSubtype",
                                    toSubtypeString(reported.chassisIdSubtype));
    }
    if (frame.managementAddressIPv4 != reported.managementAddressIPv4)
    {
        reported.managementAddressIPv4 = frame.managementAddressIPv4;
        tlvsInterface->set_property("ManagementAddressIPv4",
                                    reported.managementAddressIPv4);
    }
    if (frame.managementAddressIPv6 != reported.managementAddressIPv6)
    {
        reported.managementAddressIPv6 = frame.managementAddressIPv6;
        tlvsInterface->set_property("ManagementAddressIPv6",
                                    reported.managementAddressIPv6);
    }
    if (frame.managementAddressMAC != reported.managementAddressMAC)
    {
        reported.managementAddressMAC = frame.managementAddressMAC;
        tlvsInterface->set_property("ManagementAddressMAC",
                                    reported.managementAddressMAC);
    }
    if (frame.portId != reported.portId)
    {
        reported.portId = frame.portId;
        tlvsInterface->set_property("PortId", reported.portId);
    }
    if (frame.portIdSubtype != reported.portIdSubtype)
    {
        reported.portIdSubtype = frame.portIdSubtype;
        tlvsInterface->set_property("PortIdSubtype",
                                    toSubtypeString(reported.portIdSubtype));
    }
    if (frame.systemCapabilities != reported.systemCapabilities)
    {
        reported.systemCapabilities = frame.systemCapabilities;
        tlvsInterface->set_property(
            "SystemCapabilities",
            toCapabilityStrings(reported.systemCapabilities));
    }
    if (frame.systemDescription != reported.systemDescription)
    {
        reported.systemDescription = frame.systemDescription;
        tlvsInterface->set_property("SystemDescription",
                                    reported.systemDescription);
    }
    if (frame.systemName != reported.systemName)
    {
        reported.systemName = frame.systemName;
        tlvsInterface->set_property("SystemName", reported.systemName);
    }
}
