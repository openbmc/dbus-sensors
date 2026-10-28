/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "PcieState.hpp"

#include "NetworkAdapter.hpp"

#include <phosphor-logging/lg2.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>

namespace ocp::bulk::pcie_state
{

namespace
{

constexpr size_t faultReasonLength = 80;

// Values are little endian.
struct PcieStateData
{
    uint32_t port;
    int32_t capGen;
    int32_t capWidth;
    int32_t curGen;
    int32_t curWidth;
    uint8_t curReversed;
    uint8_t priBus;
    uint8_t secBus;
    uint8_t subBus;
    uint16_t laneMask;
    uint16_t subVendorId;
    uint16_t subDeviceId;
    uint8_t crs;
    uint8_t powerDown;
    uint32_t macUp;
    uint32_t state;
    uint32_t event;
    std::array<char, faultReasonLength> faultReason;
    std::array<char, faultReasonLength> lastFaultReason;
};

using Generations = PcieSlotInfo::Generations;

enum class PciePortState : uint32_t
{
    Off = 0,
    Down,
    MacUp,
    LinkUp,
    Up,
    Fault,
};

enum class PcieMacState : uint32_t
{
    Down = 0,
    Up,
};

constexpr std::array<Generations, 6> generations{
    Generations::Gen1, Generations::Gen2, Generations::Gen3,
    Generations::Gen4, Generations::Gen5, Generations::Gen6};

Generations generationOf(int32_t encoded)
{
    if (encoded < 1 || static_cast<size_t>(encoded) > generations.size())
    {
        return Generations::Unknown;
    }

    return generations[static_cast<size_t>(encoded) - 1];
}

PortInfo::LinkState linkStateOf(PciePortState state)
{
    switch (state)
    {
        case PciePortState::Off:
            return PortInfo::LinkState::Disabled;
        case PciePortState::Down:
        case PciePortState::MacUp:
        case PciePortState::LinkUp:
        case PciePortState::Up:
        case PciePortState::Fault:
            return PortInfo::LinkState::Enabled;
        default:
            return PortInfo::LinkState::Unknown;
    }
}

PortInfo::LinkStatus linkStatusOf(PciePortState state, PcieMacState macUp,
                                  uint8_t powerDown)
{
    if (powerDown != 0 || state == PciePortState::Off)
    {
        return PortInfo::LinkStatus::NoLink;
    }

    switch (state)
    {
        case PciePortState::Down:
        case PciePortState::Fault:
            return PortInfo::LinkStatus::LinkDown;
        case PciePortState::MacUp:
            return macUp == PcieMacState::Up ? PortInfo::LinkStatus::Training
                                             : PortInfo::LinkStatus::LinkDown;
        case PciePortState::LinkUp:
            return macUp == PcieMacState::Up ? PortInfo::LinkStatus::Starting
                                             : PortInfo::LinkStatus::LinkDown;
        case PciePortState::Up:
            return macUp == PcieMacState::Up ? PortInfo::LinkStatus::LinkUp
                                             : PortInfo::LinkStatus::LinkDown;
        default:
            return PortInfo::LinkStatus::Unknown;
    }
}

} // namespace

std::optional<PcieLink> decode(std::span<const uint8_t> data)
{
    if (data.size() != sizeof(PcieStateData))
    {
        lg2::error("PCIe state block is {SIZE} bytes, expected {EXPECTED}",
                   "SIZE", data.size(), "EXPECTED", sizeof(PcieStateData));
        return std::nullopt;
    }

    PcieStateData raw{};
    std::memcpy(&raw, data.data(), sizeof(raw));

    const PciePortState portState = static_cast<PciePortState>(raw.state);
    const PcieMacState macState =
        raw.macUp == 0 ? PcieMacState::Down : PcieMacState::Up;

    return PcieLink{
        .generationSupported = generationOf(raw.capGen),
        .generationInUse = generationOf(raw.curGen),
        .maxLanes = raw.capWidth > 0 ? static_cast<size_t>(raw.capWidth) : 0,
        .lanesInUse = raw.curWidth > 0 ? static_cast<size_t>(raw.curWidth) : 0,
        .busNumber = raw.priBus,
        .linkState = linkStateOf(portState),
        .linkStatus = linkStatusOf(portState, macState, raw.powerDown),
        .subsystemVendorId = raw.subVendorId,
        .subsystemId = raw.subDeviceId,
        .available = raw.powerDown == 0 && macState == PcieMacState::Up,
        .functional = raw.faultReason[0] == '\0',
    };
}

void update(NetworkAdapter& adapter, size_t instance,
            std::span<const uint8_t> data)
{
    const std::optional<PcieLink> link = decode(data);
    if (!link)
    {
        return;
    }

    adapter.updatePcieLink(instance, *link);
}

} // namespace ocp::bulk::pcie_state
