/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "OsfpStats.hpp"

#include "NetworkAdapter.hpp"

#include <phosphor-logging/lg2.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <optional>
#include <span>
#include <string>

namespace ocp::bulk::osfp_stats
{

namespace
{

constexpr size_t stringFieldLength = 16;

// Values are little endian.
struct OsfpData
{
    uint32_t sff8024Code;
    uint32_t cableLength;
    uint16_t fwActiveBuild;
    uint16_t fwInactiveBuild;
    int16_t temp;
    int16_t tempWarnHi;
    int16_t tempWarnLo;
    int16_t tempAlarmHi;
    int16_t tempAlarmLo;
    std::array<uint16_t, 64> vdmPrefec;
    std::array<uint16_t, 8> txPower;
    std::array<uint16_t, 8> rxPower;
    std::array<uint16_t, 8> txBias;
    std::array<uint16_t, 4> txPowerThresh;
    std::array<uint16_t, 4> txBiasThresh;
    std::array<uint16_t, 4> rxPowerThresh;
    uint16_t vcc;
    std::array<int16_t, 3> auxMon;
    int16_t customMon;
    std::array<uint16_t, 4> vccThresh;
    std::array<int16_t, 12> auxThresh;
    std::array<uint16_t, 64> vdmExt;
    uint16_t vdmExtSupportedMask;
    std::array<uint16_t, 80> vdmFerc;
    uint16_t vdmFercSupportedMask;
    std::array<char, stringFieldLength> vendor;
    std::array<char, stringFieldLength> partNum;
    std::array<char, stringFieldLength> serialNum;
    uint8_t present;
    uint8_t ready;
    uint8_t transType;
    uint8_t media;
    uint8_t mediaSubtype;
    uint8_t fwActiveValid;
    uint8_t fwActiveMajor;
    uint8_t fwActiveMinor;
    uint8_t fwInactiveValid;
    uint8_t fwInactiveMajor;
    uint8_t fwInactiveMinor;
    uint8_t tempValid;
    uint8_t tempThreshValid;
    uint8_t flagDisconnected;
    uint8_t flagNotReady;
    uint8_t flagBad;
    uint8_t flagUnsupported;
    uint8_t flagCommsFault;
    uint8_t flagPowerFault;
    uint8_t flagThermalFault;
    uint8_t losMask;
    uint8_t vdmPrefecSupportedMask;
    uint8_t modState;
    uint8_t faultCause;
    uint8_t flagsModule;
    uint8_t flagsTempVcc;
    uint8_t flagsAux12;
    uint8_t flagsAux3Custom;
    uint8_t powerClass;
    uint8_t maxPower;
    std::array<uint8_t, 8> dpState;
    std::array<uint8_t, 4> txPowerFlags;
    std::array<uint8_t, 4> txBiasFlags;
    std::array<uint8_t, 4> rxPowerFlags;
    uint8_t txFaultMask;
    uint8_t txLolMask;
    uint8_t rxLolMask;
    uint8_t txEqFaultMask;
    uint8_t rxLosMask;
    uint8_t stateValid;
    uint8_t dpStateValid;
    uint8_t laneMonValid;
    uint8_t laneThreshValid;
    uint8_t supplyValid;
    uint8_t supplyThreshValid;
    uint8_t vdmExtValidAny;
    uint8_t vdmFercValidAny;
    uint8_t vdmFrozen;
    std::array<uint8_t, 64> vdmPrefecValid;
    std::array<uint8_t, 64> vdmExtValid;
    std::array<uint8_t, 80> vdmFercValid;
};

// Fields are space padded and have no terminator when full, so one ends at
// the first byte that is not printable ASCII.
std::string readStringField(const std::array<char, stringFieldLength>& field)
{
    const auto* const end = std::ranges::find_if_not(field, [](char byte) {
        return byte >= 0x20 && byte < 0x7F;
    });

    std::string value(field.begin(), end);
    value.erase(value.find_last_not_of(' ') + 1);
    return value;
}

std::string readFirmwareVersion(const OsfpData& osfp)
{
    if (osfp.fwActiveValid == 0)
    {
        return {};
    }

    return std::format("{}.{}.{}", osfp.fwActiveMajor, osfp.fwActiveMinor,
                       osfp.fwActiveBuild);
}

using Type = TransceiverInfo::TransceiverType;
using Mgmt = TransceiverInfo::ManagementInterface;
using Medium = TransceiverInfo::MediumType;

struct TransceiverTypeInfo
{
    Type type;
    Mgmt managementInterface;
};

constexpr std::array<TransceiverTypeInfo, 17> transceiverTypes{{
    {Type::Unknown, Mgmt::Unknown},
    {Type::Unknown, Mgmt::Unknown},
    {Type::Unknown, Mgmt::Unknown},
    {Type::SFP, Mgmt::SFP},
    {Type::SFPPlus, Mgmt::SFP},
    {Type::SFP28, Mgmt::SFP},
    {Type::SFP56, Mgmt::SFP},
    {Type::SFP112, Mgmt::CMIS},
    {Type::SFPDD, Mgmt::CMIS},
    {Type::SFPDD112, Mgmt::CMIS},
    {Type::QSFP, Mgmt::SFP},
    {Type::QSFPPlus, Mgmt::SFP},
    {Type::QSFP28, Mgmt::SFP},
    {Type::QSFP56, Mgmt::SFP},
    {Type::QSFP112, Mgmt::CMIS},
    {Type::QSFPDD, Mgmt::CMIS},
    {Type::OSFP, Mgmt::CMIS},
}};

// The firmware media codes, indexed by value.
constexpr std::array<Medium, 4> mediumTypes{
    Medium::Unknown, Medium::Copper, Medium::FiberOptic, Medium::Unknown};

// Lane monitor scaling
constexpr double powerWattsPerCount = 1e-7;
constexpr double biasAmperesPerCount = 2e-6;
constexpr double supplyVoltsPerCount = 1e-4;

// The firmware reports module readiness, not MAC link state, so a present and
// ready module counts as a link.
PortInfo::LinkStatus readLinkStatus(const OsfpData& osfp)
{
    if (osfp.stateValid == 0)
    {
        return PortInfo::LinkStatus::Unknown;
    }

    return osfp.ready != 0 ? PortInfo::LinkStatus::LinkUp
                           : PortInfo::LinkStatus::Starting;
}

std::optional<std::array<LaneOptics, osfpLaneCount>> readLanes(
    const OsfpData& osfp)
{
    if (osfp.laneMonValid == 0)
    {
        return std::nullopt;
    }

    std::array<LaneOptics, osfpLaneCount> lanes{};
    for (size_t lane = 0; lane < lanes.size(); lane++)
    {
        lanes[lane] = {
            .txOutputPowerWatts = osfp.txPower[lane] * powerWattsPerCount,
            .rxInputPowerWatts = osfp.rxPower[lane] * powerWattsPerCount,
            .txBiasCurrentAmps = osfp.txBias[lane] * biasAmperesPerCount,
        };
    }
    return lanes;
}

} // namespace

std::optional<Transceiver> decode(std::span<const uint8_t> data)
{
    if (data.size() != sizeof(OsfpData))
    {
        lg2::error("OSFP struct is {SIZE} bytes, expected {EXPECTED}", "SIZE",
                   data.size(), "EXPECTED", sizeof(OsfpData));
        return std::nullopt;
    }

    OsfpData osfp{};
    std::memcpy(&osfp, data.data(), sizeof(osfp));

    if (osfp.present == 0 || osfp.flagDisconnected != 0)
    {
        Transceiver absent;
        absent.linkStatus = PortInfo::LinkStatus::NoLink;
        return absent;
    }

    return Transceiver{
        .present = true,
        .functional = osfp.stateValid == 0 || osfp.ready != 0,
        .manufacturer = readStringField(osfp.vendor),
        .partNumber = readStringField(osfp.partNum),
        .serialNumber = readStringField(osfp.serialNum),
        .firmwareVersion = readFirmwareVersion(osfp),
        .type = osfp.transType < transceiverTypes.size()
                    ? transceiverTypes[osfp.transType].type
                    : Type::Unknown,
        .mediumType = osfp.media < mediumTypes.size() ? mediumTypes[osfp.media]
                                                      : Medium::Unknown,
        .managementInterface =
            osfp.transType < transceiverTypes.size()
                ? transceiverTypes[osfp.transType].managementInterface
                : Mgmt::Unknown,
        .linkStatus = readLinkStatus(osfp),
        .cableLengthMeters = osfp.cableLength,
        .lanes = readLanes(osfp),
        .supplyVoltageVolts =
            osfp.supplyValid != 0
                ? std::optional<double>(osfp.vcc * supplyVoltsPerCount)
                : std::nullopt,
    };
}

void update(NetworkAdapter& adapter, size_t instance,
            std::span<const uint8_t> data)
{
    const std::optional<Transceiver> transceiver = decode(data);
    if (!transceiver)
    {
        return;
    }

    adapter.updateTransceiver(instance, *transceiver);
}

} // namespace ocp::bulk::osfp_stats
