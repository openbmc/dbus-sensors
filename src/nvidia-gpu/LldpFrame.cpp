/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "LldpFrame.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <span>
#include <string>
#include <vector>

namespace lldp
{

namespace
{

// Type-length-value field types, from IEEE 802.1AB-2009 table 8-1.
constexpr uint8_t tlvEndOfLldpdu = 0;
constexpr uint8_t tlvChassisId = 1;
constexpr uint8_t tlvPortId = 2;
constexpr uint8_t tlvTimeToLive = 3;
constexpr uint8_t tlvSystemName = 5;
constexpr uint8_t tlvSystemDescription = 6;
constexpr uint8_t tlvSystemCapabilities = 7;
constexpr uint8_t tlvManagementAddress = 8;

// A field is introduced by seven bits of type and nine of length, so the two
// share the first of the two header bytes.
constexpr size_t tlvHeaderSize = 2;
constexpr uint8_t tlvLengthHighBitMask = 0x01;

// A subtype byte precedes the identifier in the chassis and port fields.
constexpr size_t identifierMinimumSize = 2;

// The capabilities field holds what the device supports and what of that it has
// enabled, two bytes each. Only the first pair describes the device itself.
constexpr size_t capabilitiesSize = 4;

// IEEE 802.1AB-2009 section 8.5.9 fixes what follows a management address: one
// byte of interface numbering subtype, four of interface number, one of object
// identifier length.
constexpr size_t managementAddressTrailerSize = 6;

// Address families, numbered by IANA, as the management address subtype uses
// them.
constexpr uint8_t addressFamilyIPv4 = 1;
constexpr uint8_t addressFamilyIPv6 = 2;
constexpr uint8_t addressFamilyMac = 6;

constexpr size_t ipv4AddressSize = 4;
constexpr size_t ipv6AddressSize = 16;
constexpr size_t macAddressSize = 6;

// Which mandatory field the unit must carry next. IEEE 802.1AB-2009 section
// 8.4 requires the first three in this order.
enum class Mandatory : uint8_t
{
    ChassisId,
    PortId,
    TimeToLive,
    Done,
};

std::string toHexadecimalOctets(std::span<const uint8_t> bytes)
{
    std::string text;
    for (const uint8_t byte : bytes)
    {
        if (!text.empty())
        {
            text.push_back(':');
        }
        std::format_to(std::back_inserter(text), "{:02X}", byte);
    }
    return text;
}

// Whether bytes is well formed UTF-8, by the encoding's own rules: a leading
// byte names how many continuation bytes follow, over-long forms are excluded
// by a minimum for each length, and the surrogate range is not encodable.
bool isUtf8(std::span<const uint8_t> bytes)
{
    size_t index = 0;
    while (index < bytes.size())
    {
        const uint8_t lead = bytes[index];
        size_t continuations = 0;
        uint32_t point = 0;

        if (lead < 0x80)
        {
            index += 1;
            continue;
        }
        if ((lead & 0xE0) == 0xC0)
        {
            continuations = 1;
            point = lead & 0x1FU;
        }
        else if ((lead & 0xF0) == 0xE0)
        {
            continuations = 2;
            point = lead & 0x0FU;
        }
        else if ((lead & 0xF8) == 0xF0)
        {
            continuations = 3;
            point = lead & 0x07U;
        }
        else
        {
            return false;
        }

        if (index + continuations >= bytes.size())
        {
            return false;
        }
        for (size_t offset = 1; offset <= continuations; ++offset)
        {
            const uint8_t next = bytes[index + offset];
            if ((next & 0xC0) != 0x80)
            {
                return false;
            }
            point = (point << 6) | (next & 0x3FU);
        }

        static constexpr std::array<uint32_t, 4> minimum{0, 0x80, 0x800,
                                                         0x10000};
        if (point < minimum.at(continuations) || point > 0x10FFFF ||
            (point >= 0xD800 && point <= 0xDFFF))
        {
            return false;
        }
        index += continuations + 1;
    }
    return true;
}

// A device is free to pad a text field out with nulls, and nothing past the
// first null is part of the text in any case. What is left has to be UTF-8,
// which is all D-Bus can carry, so text that is not is dropped here rather
// than left to fail on the bus.
std::string toText(std::span<const uint8_t> bytes)
{
    const auto end = std::ranges::find(bytes, 0);
    const std::span<const uint8_t> text{bytes.begin(), end};
    if (!isUtf8(text))
    {
        return {};
    }
    return {text.begin(), text.end()};
}

IdSubtype toChassisIdSubtype(uint8_t value)
{
    switch (value)
    {
        case 1:
            return IdSubtype::ChassisComp;
        case 2:
            return IdSubtype::IfAlias;
        case 3:
            return IdSubtype::PortComp;
        case 4:
            return IdSubtype::MacAddr;
        case 5:
            return IdSubtype::NetworkAddr;
        case 6:
            return IdSubtype::IfName;
        case 7:
            return IdSubtype::LocalAssign;
        default:
            return IdSubtype::NotTransmitted;
    }
}

IdSubtype toPortIdSubtype(uint8_t value)
{
    switch (value)
    {
        case 1:
            return IdSubtype::IfAlias;
        case 2:
            return IdSubtype::PortComp;
        case 3:
            return IdSubtype::MacAddr;
        case 4:
            return IdSubtype::NetworkAddr;
        case 5:
            return IdSubtype::IfName;
        case 6:
            return IdSubtype::AgentId;
        case 7:
            return IdSubtype::LocalAssign;
        default:
            return IdSubtype::NotTransmitted;
    }
}

// An identifier whose subtype names something a person wrote is rendered as
// that text; everything else becomes hexadecimal octets, the form the Redfish
// Port schema documents for these properties. A subtype neither table names is
// left empty, because NotTransmitted is the only thing this interface can say
// about it and that means the identifier is not there.
std::string toIdentifier(IdSubtype subtype, std::span<const uint8_t> value)
{
    switch (subtype)
    {
        case IdSubtype::AgentId:
        case IdSubtype::ChassisComp:
        case IdSubtype::IfAlias:
        case IdSubtype::IfName:
        case IdSubtype::LocalAssign:
        case IdSubtype::PortComp:
            return toText(value);
        case IdSubtype::MacAddr:
        case IdSubtype::NetworkAddr:
            return toHexadecimalOctets(value);
        case IdSubtype::NotTransmitted:
        default:
            return {};
    }
}

std::vector<SystemCapability> toCapabilities(uint16_t bits)
{
    static constexpr std::array<SystemCapability, 8> byBit{
        SystemCapability::Other,
        SystemCapability::Repeater,
        SystemCapability::Bridge,
        SystemCapability::WLANAccessPoint,
        SystemCapability::Router,
        SystemCapability::Telephone,
        SystemCapability::DOCSISCableDevice,
        SystemCapability::Station,
    };

    std::vector<SystemCapability> capabilities;
    for (size_t bit = 0; bit < byBit.size(); ++bit)
    {
        if ((bits & (1U << bit)) != 0)
        {
            capabilities.push_back(byBit.at(bit));
        }
    }

    // A device that carried the field but claimed nothing in it is saying
    // something, so that alone becomes None; a device that carried no field at
    // all leaves the vector empty. A device claiming only functions above the
    // bits this enumeration names also leaves it empty, which is the closest
    // this interface can come to what it said.
    if (bits == 0)
    {
        capabilities.push_back(SystemCapability::None);
    }
    return capabilities;
}

// Renders an address of a known family, or nothing at all when the family and
// the length disagree. Only the first address of each family is kept: the rest
// belong in properties this interface does not carry.
void readManagementAddress(uint8_t family, std::span<const uint8_t> address,
                           Frame& frame)
{
    if (family == addressFamilyIPv4 && address.size() == ipv4AddressSize &&
        frame.managementAddressIPv4.empty())
    {
        std::array<char, INET_ADDRSTRLEN> text{};
        if (inet_ntop(AF_INET, address.data(), text.data(), text.size()) !=
            nullptr)
        {
            frame.managementAddressIPv4 = text.data();
        }
    }
    else if (family == addressFamilyIPv6 && address.size() == ipv6AddressSize &&
             frame.managementAddressIPv6.empty())
    {
        std::array<char, INET6_ADDRSTRLEN> text{};
        if (inet_ntop(AF_INET6, address.data(), text.data(), text.size()) !=
            nullptr)
        {
            frame.managementAddressIPv6 = text.data();
        }
    }
    else if (family == addressFamilyMac && address.size() == macAddressSize &&
             frame.managementAddressMAC.empty())
    {
        frame.managementAddressMAC = toHexadecimalOctets(address);
    }
}

// IEEE 802.1AB-2009 section 8.5.9. The leading byte counts the subtype byte
// together with the address, and the object identifier that ends the field is
// itself length-prefixed, so the whole thing has to add up.
void readManagementAddressField(std::span<const uint8_t> value, Frame& frame)
{
    if (value.empty())
    {
        return;
    }

    const size_t addressStringLength = value[0];
    if (addressStringLength == 0)
    {
        return;
    }

    const size_t addressBlockSize = 1 + addressStringLength;
    if (value.size() < addressBlockSize + managementAddressTrailerSize)
    {
        return;
    }

    const size_t objectIdentifierLength =
        value[addressBlockSize + managementAddressTrailerSize - 1];
    if (value.size() != addressBlockSize + managementAddressTrailerSize +
                            objectIdentifierLength)
    {
        return;
    }

    readManagementAddress(value[1], value.subspan(2, addressStringLength - 1),
                          frame);
}

// An optional field the sender got wrong costs only itself. IEEE 802.1AB has
// a receiver discard such a field and go on with the rest of the unit, which
// is what a consumer wants here too: a neighbor missing one value is far more
// use than no neighbor at all.
void readOptionalField(uint8_t type, std::span<const uint8_t> value,
                       Frame& frame)
{
    switch (type)
    {
        case tlvSystemName:
            frame.systemName = toText(value);
            break;
        case tlvSystemDescription:
            frame.systemDescription = toText(value);
            break;
        case tlvSystemCapabilities:
            if (value.size() == capabilitiesSize)
            {
                frame.systemCapabilities = toCapabilities(
                    static_cast<uint16_t>((value[0] << 8) | value[1]));
            }
            break;
        case tlvManagementAddress:
            readManagementAddressField(value, frame);
            break;
        default:
            // Fields this interface does not carry, and types a later revision
            // of the standard may give a meaning to.
            break;
    }
}

} // namespace

bool parse(std::span<const uint8_t> pdu, Frame& frame)
{
    frame = Frame{};

    Mandatory expected = Mandatory::ChassisId;
    size_t cursor = 0;

    while (cursor + tlvHeaderSize <= pdu.size())
    {
        const uint8_t first = pdu[cursor];
        const uint8_t second = pdu[cursor + 1];
        const auto type = static_cast<uint8_t>(first >> 1);
        const size_t length =
            (static_cast<size_t>(first & tlvLengthHighBitMask) << 8) | second;
        cursor += tlvHeaderSize;

        if (cursor + length > pdu.size())
        {
            return false;
        }
        const std::span<const uint8_t> value = pdu.subspan(cursor, length);
        cursor += length;

        if (type == tlvEndOfLldpdu)
        {
            // Section 8.5.1: the unit stops here. Whatever follows was not
            // written by the sender: a link pads a short frame out to its own
            // minimum length, and a transport may pad again to reach an
            // alignment boundary. Neither is visible from here and the two
            // together run to tens of bytes, so the run is not bounded. It
            // only has to be the zeros padding is made of, rather than data
            // that arrived truncated or scrambled.
            const std::span<const uint8_t> trailing = pdu.subspan(cursor);
            return length == 0 && expected == Mandatory::Done &&
                   std::ranges::all_of(trailing, [](uint8_t byte) {
                       return byte == 0;
                   });
        }

        switch (type)
        {
            case tlvChassisId:
                if (expected != Mandatory::ChassisId ||
                    length < identifierMinimumSize)
                {
                    return false;
                }
                frame.chassisIdSubtype = toChassisIdSubtype(value[0]);
                frame.chassisId =
                    toIdentifier(frame.chassisIdSubtype, value.subspan(1));
                expected = Mandatory::PortId;
                break;

            case tlvPortId:
                if (expected != Mandatory::PortId ||
                    length < identifierMinimumSize)
                {
                    return false;
                }
                frame.portIdSubtype = toPortIdSubtype(value[0]);
                frame.portId =
                    toIdentifier(frame.portIdSubtype, value.subspan(1));
                expected = Mandatory::TimeToLive;
                break;

            case tlvTimeToLive:
                // How long the neighbor's data stays good for. The device
                // ages its own buffer on this, so only the shape is checked.
                if (expected != Mandatory::TimeToLive || length != 2)
                {
                    return false;
                }
                expected = Mandatory::Done;
                break;

            default:
                if (expected != Mandatory::Done)
                {
                    return false;
                }
                readOptionalField(type, value, frame);
                break;
        }
    }

    // Ran out of bytes without reaching an end marker.
    return false;
}

} // namespace lldp
