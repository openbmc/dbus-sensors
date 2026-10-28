/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "RdmaStats.hpp"

#include "NetworkAdapter.hpp"

#include <phosphor-logging/lg2.hpp>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

namespace ocp::bulk::rdma_stats
{

namespace
{

using Unit = MetricInfo::Unit;

// Values are little endian.
struct RdmaData
{
    uint64_t txUcastBytes;
    uint64_t txUcastPkts;
    uint64_t txMcastBytes;
    uint64_t txMcastPkts;
    uint64_t rxUcastBytes;
    uint64_t rxUcastPkts;
    uint64_t rxMcastBytes;
    uint64_t rxMcastPkts;
    uint64_t txReadReqPkts;
    uint64_t txSendPkts;
    uint64_t txWritePkts;
    uint64_t txCnpPkts;
    uint64_t rxCnpPkts;
    uint64_t rxEcnPkts;
    uint64_t reqRxPktSeqErr;
    uint64_t reqRxRnrRetryErr;
    uint64_t reqRxRmtAccErr;
    uint64_t reqRxRmtReqErr;
    uint64_t reqRxOperErr;
    uint64_t reqRxImplNakSeqErr;
    uint64_t reqRxCqeErr;
    uint64_t reqRxCqeFlush;
    uint64_t reqRxDupResponse;
    uint64_t reqRxInvalPkts;
    uint64_t reqTxLocAccErr;
    uint64_t reqTxLocOperErr;
    uint64_t reqTxMemMgmtErr;
    uint64_t reqTxRetryExcdErr;
    uint64_t reqTxLocSglInvErr;
    uint64_t respRxDupRequest;
    uint64_t respRxOutofBuf;
    uint64_t respRxOutofSeq;
    uint64_t respRxCqeErr;
    uint64_t respRxCqeFlush;
    uint64_t respRxLocLenErr;
    uint64_t respRxInvalRequest;
    uint64_t respRxLocOperErr;
    uint64_t respRxOutofAtomic;
    uint64_t respTxPktSeqErr;
    uint64_t respTxRmtInvalReqErr;
    uint64_t respTxRmtAccErr;
    uint64_t respTxRmtOperErr;
    uint64_t respTxRnrRetryErr;
    uint64_t respTxLocSglInvErr;
    uint64_t txAckTimeout;
    uint64_t qpCreate;
    uint64_t qpCreateFail;
    uint64_t qpDelete;
    uint64_t qpDeleteFail;
    uint64_t cqCreate;
    uint64_t cqCreateFail;
    uint64_t cqDelete;
    uint64_t cqDeleteFail;
    uint64_t mrCreate;
    uint64_t mrCreateFail;
    uint64_t mrDelete;
    uint64_t mrDeleteFail;
    uint64_t atsTrans;
    uint64_t atsTransFail;
    uint64_t atsInval;
    uint64_t atsInvalFail;
};

} // namespace

std::vector<PortCounter> decode(std::span<const uint8_t> data)
{
    if (data.size() != sizeof(RdmaData))
    {
        lg2::error("RDMA block is {SIZE} bytes, expected {EXPECTED}", "SIZE",
                   data.size(), "EXPECTED", sizeof(RdmaData));
        return {};
    }

    RdmaData rdma{};
    std::memcpy(&rdma, data.data(), sizeof(rdma));

    return {
        {"rdma_tx_read_requests", Unit::Count, rdma.txReadReqPkts},
        {"rdma_tx_send_requests", Unit::Count, rdma.txSendPkts},
        {"rdma_tx_write_requests", Unit::Count, rdma.txWritePkts},
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

} // namespace ocp::bulk::rdma_stats
