/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "XrmacStats.hpp"

#include "NetworkAdapter.hpp"

#include <phosphor-logging/lg2.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

namespace ocp::bulk::xrmac_stats
{

namespace
{

using Unit = MetricInfo::Unit;

constexpr size_t fecLaneCount = 32;
constexpr size_t fecErrorBinCount = 16;

// Values are little endian.
struct XrmacData
{
    uint64_t txPkts;
    uint64_t txPausePkts;
    uint64_t txGoodPkts;
    uint64_t txControlPkts;
    uint64_t txUnicastPkts;
    uint64_t txMulticastPkts;
    uint64_t txBroadcastPkts;
    uint64_t txBytes;
    uint64_t txBadBytes;
    uint64_t txGoodBytes;
    uint64_t txLt64Pkts;
    uint64_t tx64Pkts;
    uint64_t tx65To127Pkts;
    uint64_t tx128To255Pkts;
    uint64_t tx256To511Pkts;
    uint64_t tx512To1023Pkts;
    uint64_t tx1024To15xxPkts;
    uint64_t tx15xxToJumboPkts;
    uint64_t txGtJumboPkts;
    uint64_t txBadFcsPkts;
    uint64_t txGoodFcsPkts;
    uint64_t rxPkts;
    uint64_t rxPausePkts;
    uint64_t rxGoodPkts;
    uint64_t rxBadPkts;
    uint64_t rxControlPkts;
    uint64_t rxUnicastPkts;
    uint64_t rxMulticastPkts;
    uint64_t rxBroadcastPkts;
    uint64_t rxBytes;
    uint64_t rxBadBytes;
    uint64_t rxGoodBytes;
    uint64_t rx64Pkts;
    uint64_t rx65To127Pkts;
    uint64_t rx128To255Pkts;
    uint64_t rx256To511Pkts;
    uint64_t rx512To1023Pkts;
    uint64_t rx1024To15xxPkts;
    uint64_t rx15xxToJumboPkts;
    uint64_t rxGtJumboPkts;
    uint64_t rxUndersizePkts;
    uint64_t rxBadFcsPkts;
    uint64_t rxGoodFcsPkts;
    uint64_t rxOverflowPkts;
    uint64_t rxSymbolErrorPkts;
    uint64_t rxAlignErrorPkts;
    uint64_t rxLengthErrorPkts;
    uint64_t rxInternalErrorPkts;
    uint64_t rxJabberPkts;
    uint64_t rxNodescDrops;
    uint64_t rxLanes01CharErr;
    uint64_t rxLanes23CharErr;
    uint64_t rxLanes01DispErr;
    uint64_t rxLanes23DispErr;
    uint64_t rxMatchFault;
    uint64_t rx1024To1518Pkts;
    uint64_t rx1519To1522Pkts;
    uint64_t rx1523To1548Pkts;
    uint64_t rx1549To2047Pkts;
    uint64_t rx2048To4095Pkts;
    uint64_t rx4096To8191Pkts;
    uint64_t rx8192To9215Pkts;
    uint64_t rxVlanGoodPkts;
    uint64_t rxTruncated;
    uint64_t rxStompedFcs;
    uint64_t tx1024To1518Pkts;
    uint64_t tx1519To1522Pkts;
    uint64_t tx1523To1548Pkts;
    uint64_t tx1549To2047Pkts;
    uint64_t tx2048To4095Pkts;
    uint64_t tx4096To8191Pkts;
    uint64_t tx8192To9215Pkts;
    uint64_t txVlanPkts;
    uint64_t fecUncorrectedErrors;
    uint64_t fecCorrectedErrors;
    uint64_t fecCwCounter;
    uint64_t fecUncorrectedCwb;
    uint64_t fecCorrectedCwb;
    uint64_t fecCwbCounter;
    std::array<uint64_t, fecLaneCount> fecCorrectedSymbolsPerLane;
    std::array<uint64_t, fecErrorBinCount> fecCodewordsPerErrorCount;
    std::array<uint64_t, fecLaneCount> fecCorrectedBitsPerLane;
    uint64_t fecCorrectedBits;
    uint64_t fecBits;
};

// The counters are not sampled together, so good can briefly exceed total.
uint64_t errorsOf(uint64_t total, uint64_t good)
{
    return total > good ? total - good : 0;
}

} // namespace

std::vector<PortCounter> decode(std::span<const uint8_t> data)
{
    if (data.size() != sizeof(XrmacData))
    {
        lg2::error("XRMAC block is {SIZE} bytes, expected {EXPECTED}", "SIZE",
                   data.size(), "EXPECTED", sizeof(XrmacData));
        return {};
    }

    XrmacData xrmac{};
    std::memcpy(&xrmac, data.data(), sizeof(xrmac));

    return {
        {"tx_frames", Unit::Count, xrmac.txPkts},
        {"tx_errors", Unit::Count, errorsOf(xrmac.txPkts, xrmac.txGoodPkts)},
        {"tx_unicast_frames", Unit::Count, xrmac.txUnicastPkts},
        {"tx_multicast_frames", Unit::Count, xrmac.txMulticastPkts},
        {"tx_broadcast_frames", Unit::Count, xrmac.txBroadcastPkts},
        {"tx_bytes", Unit::Bytes, xrmac.txBytes},
        {"rx_frames", Unit::Count, xrmac.rxPkts},
        {"rx_errors", Unit::Count, errorsOf(xrmac.rxPkts, xrmac.rxGoodPkts)},
        {"rx_discards", Unit::Count, xrmac.rxBadPkts},
        {"rx_unicast_frames", Unit::Count, xrmac.rxUnicastPkts},
        {"rx_multicast_frames", Unit::Count, xrmac.rxMulticastPkts},
        {"rx_broadcast_frames", Unit::Count, xrmac.rxBroadcastPkts},
        {"rx_bytes", Unit::Bytes, xrmac.rxBytes},
        {"rx_undersize_frames", Unit::Count, xrmac.rxUndersizePkts},
        {"rx_fcs_errors", Unit::Count, xrmac.rxBadFcsPkts},
        {"rx_oversize_frames", Unit::Count, xrmac.rxOverflowPkts},
        {"rx_frame_alignment_errors", Unit::Count, xrmac.rxAlignErrorPkts},
        {"rx_false_carrier_errors", Unit::Count, xrmac.rxMatchFault},
    };
}

void update(NetworkAdapter& adapter, size_t instance,
            std::span<const uint8_t> data)
{
    const std::vector<PortCounter> counters = decode(data);
    if (counters.empty())
    {
        return;
    }

    adapter.updatePortCounters(instance, counters);
}

} // namespace ocp::bulk::xrmac_stats
