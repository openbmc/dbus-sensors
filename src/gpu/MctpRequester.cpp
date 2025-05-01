/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "MctpRequester.hpp"

#include <linux/mctp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <OcpMctpVdm.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/local/datagram_protocol.hpp>
#include <boost/asio/steady_timer.hpp>
#include <phosphor-logging/lg2.hpp>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace mctp
{

MctpRequester::MctpRequester(boost::asio::io_context& ctx, uint8_t msgType) :
    ctx(ctx), sockfd(socket(AF_MCTP, SOCK_DGRAM, 0)), mctpSocket(ctx),
    msgType(msgType)
{
    if (sockfd < 0)
    {
        lg2::error("Failed to create MCTP socket");
        return;
    }

    boost::system::error_code ec;
    mctpSocket.assign(boost::asio::local::datagram_protocol{}, sockfd, ec);

    if (ec)
    {
        lg2::error(
            "MctpRequester failed to connect to the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());
        close(sockfd);
        return;
    }

    mctpSocket.non_blocking(true);
    asyncRecv();
}

void MctpRequester::asyncRecv()
{
    std::vector<uint8_t> respMsg{};
    mctpSocket.async_receive(
        boost::asio::buffer(respMsg), MSG_PEEK | MSG_TRUNC,
        [this](const boost::system::error_code& ec, size_t peekedLength) {
            if (ec)
            {
                lg2::error(
                    "MctpRequester failed to receive data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
                    "EC", ec.value(), "ER", ec.message());
                processPendingRequests();
                asyncRecv();
                return;
            }

            struct sockaddr sockAddr{};
            struct sockaddr_mctp respAddr{};
            socklen_t addrlen = sizeof(respAddr);
            size_t receivedLength = 0;

            std::vector<uint8_t> fullRespMsg(peekedLength);

            receivedLength = recvfrom(sockfd, fullRespMsg.data(), peekedLength,
                                      MSG_TRUNC, &sockAddr, &addrlen);

            std::memcpy(&respAddr, &sockAddr, sizeof(respAddr));

            if (receivedLength <= 0)
            {
                lg2::error(
                    "MctpRequester: Failed to receive message - Error={EC}.",
                    "EC", receivedLength);
                processPendingRequests();
                asyncRecv();
                return;
            }

            if (receivedLength <
                sizeof(ocp::accelerator_management::BindingPciVid))
            {
                lg2::error(
                    "MctpRequester: Received Message of Invalid length - {LEN}",
                    "LEN", receivedLength);
                processPendingRequests();
                asyncRecv();
                return;
            }

            if (respAddr.smctp_type != msgType)
            {
                lg2::error(
                    "MctpRequester: Message type mismatch - expected={MT}, received={RM}",
                    "MT", msgType, "RM", respAddr.smctp_type);
                processPendingRequests();
                asyncRecv();
                return;
            }

            mctp_eid_t eid = respAddr.smctp_addr.s_addr;

            if (!hasActiveRequest(eid))
            {
                lg2::error(
                    "MctpRequester: No active request found for EID={EID}, "
                    "Either this is a response for a timed out request"
                    "or we received a response for a request that was not"
                    "sent out by us",
                    "EID", eid);
                processPendingRequests();
                asyncRecv();
                return;
            }

            const auto [instanceId, callback] = activeCallbacks[eid];

            activeTimers[eid]->cancel();
            cleanupActiveRequest(eid);

            ocp::accelerator_management::BindingPciVid respHdr{};
            std::memcpy(&respHdr, fullRespMsg.data(),
                        sizeof(ocp::accelerator_management::BindingPciVid));

            if (instanceId != respHdr.instance_id)
            {
                lg2::error(
                    "MctpRequester: Instance ID mismatch for EID={EID} - expected={IID}, received={RIID}",
                    "EID", eid, "IID", instanceId, "RIID",
                    static_cast<uint8_t>(respHdr.instance_id));
                callback(-5, Response{});
                processPendingRequests(eid);
                asyncRecv();
                return;
            }

            callback(0, std::move(fullRespMsg));

            processPendingRequests(eid);
            asyncRecv();
        });
}

void MctpRequester::processPendingRequests()
{
    for (auto& [eid, _] : pendingRequests)
    {
        processPendingRequests(eid);
    }
}

void MctpRequester::processPendingRequests(mctp_eid_t eid)
{
    using namespace std::chrono_literals;

    if (hasActiveRequest(eid))
    {
        return;
    }

    if (!hasPendingRequests(eid))
    {
        return;
    }

    const auto [reqMsg, callback] = pendingRequests[eid].front();
    const auto* req =
        std::bit_cast<const ocp::accelerator_management::Message*>(
            reqMsg.data());
    pendingRequests[eid].pop();

    struct sockaddr_mctp addr{};
    addr.smctp_family = AF_MCTP;
    addr.smctp_addr.s_addr = eid;
    addr.smctp_type = msgType;
    addr.smctp_tag = MCTP_TAG_OWNER;

    // Create address structure
    struct sockaddr sockAddr{};
    std::memcpy(&sockAddr, &addr, sizeof(addr));

    // Send message
    ssize_t rc = sendto(sockfd, reqMsg.data(), reqMsg.size(), 0, &sockAddr,
                        sizeof(addr));
    if (rc < 0)
    {
        lg2::error(
            "MctpRequester failed send data to the MCTP Socket - Error={EC}.",
            "EC", rc);
        callback(rc, Response{});
        boost::asio::post(ctx, [this, eid]() { processPendingRequests(eid); });
        return;
    }

    // Set up async receive with timeout
    auto timer = std::make_shared<boost::asio::steady_timer>(ctx);
    timer->expires_after(2s);

    activeCallbacks.emplace(eid, std::make_pair(req->hdr.instance_id, callback));
    activeTimers.emplace(eid, timer);

    // Set up handler for when the timer expires
    timer->async_wait(
        [callback, timer, eid, this](const boost::system::error_code& ec) {
            if (ec != boost::asio::error::operation_aborted)
            {
                callback(-1, Response{});
                cleanupActiveRequest(eid);
                processPendingRequests(eid);
            }
        });
}

bool MctpRequester::enqueueRequest(mctp_eid_t eid, Request&& reqMsg,
                                   ResponseCallback&& callback)
{
    if (reqMsg.size() < sizeof(ocp::accelerator_management::BindingPciVid))
    {
        lg2::error(
            "MctpRequester: Message too small - expected={ES}, received={RS}",
            "ES", sizeof(ocp::accelerator_management::BindingPciVid), "RS",
            reqMsg.size());
        return false;
    }

    if (!pendingRequests.contains(eid))
    {
        pendingRequests[eid] = {};
    }

    pendingRequests[eid].emplace(std::move(reqMsg), std::move(callback));
    return true;
}

void MctpRequester::sendRecvMsg(mctp_eid_t eid, Request reqMsg,
                                ResponseCallback callback)
{
    if (!enqueueRequest(eid, std::move(reqMsg), std::move(callback)))
    {
        callback(-2, Response{});
        return;
    }
    processPendingRequests(eid);
}

bool MctpRequester::hasPendingRequests(mctp_eid_t eid) const
{
    return pendingRequests.contains(eid) && !pendingRequests.at(eid).empty();
}

bool MctpRequester::hasActiveRequest(mctp_eid_t eid) const
{
    return (activeTimers.contains(eid) && activeCallbacks.contains(eid));
}

void MctpRequester::cleanupActiveRequest(mctp_eid_t eid)
{
    activeTimers.erase(eid);
    activeCallbacks.erase(eid);
}

} // namespace mctp
