/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "MctpRequester.hpp"

#include "OcpMctpVdm.hpp"

#include <linux/mctp.h>
#include <sys/socket.h>
#include <unistd.h>

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
    static std::vector<uint8_t> tmp{};
    mctpSocket.async_receive(
        boost::asio::buffer(tmp), MSG_PEEK | MSG_TRUNC,
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

            Response resp(peekedLength);

            receivedLength = recvfrom(sockfd, resp.data(), peekedLength,
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

            if (peekedLength != receivedLength)
            {
                lg2::error(
                    "MctpRequester: Received message of unexpected length - expected={EL}, received={RL}",
                    "EL", peekedLength, "RL", receivedLength);
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

            const mctp_eid_t eid = respAddr.smctp_addr.s_addr;

            handleResponse(eid, resp);
        });
}

inline void MctpRequester::handleResponse(const mctp_eid_t eid,
                                          const Response& response)
{
    if (!hasActiveRequest(eid))
    {
        lg2::error("MctpRequester: No active request found for EID={EID}, "
                   "Either this is a response for a timed out request"
                   "or we received a response for a request that was not"
                   "sent out by us",
                   "EID", eid);
        processPendingRequests();
        asyncRecv();
        return;
    }

    std::unique_ptr<Request> req = std::move(activeRequests[eid]);

    req->timer->cancel();
    cleanupActiveRequest(eid);

    const auto* respHdr =
        std::bit_cast<const ocp::accelerator_management::BindingPciVid*>(
            response.data());

    if (req->instanceId != respHdr->instance_id)
    {
        lg2::error(
            "MctpRequester: Instance ID mismatch for EID={EID} - expected={IID}, received={RIID}",
            "EID", eid, "IID", req->instanceId, "RIID",
            static_cast<uint8_t>(respHdr->instance_id));

        req->callback(-5, Response{});

        processPendingRequests(eid);
        asyncRecv();

        return;
    }

    req->callback(0, response);

    processPendingRequests(eid);
    asyncRecv();
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

    if (hasActiveRequest(eid) || !hasPendingRequests(eid))
    {
        return;
    }

    std::unique_ptr<Request> req = std::move(pendingRequests[eid].front());
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
    ssize_t rc = sendto(sockfd, req->vec.data(), req->size(), 0, &sockAddr,
                        sizeof(addr));
    if (rc < 0)
    {
        lg2::error(
            "MctpRequester failed send data to the MCTP Socket - Error={EC}.",
            "EC", rc);
        req->callback(rc, Response{});
        boost::asio::post(ctx, [this, eid]() { processPendingRequests(eid); });
        return;
    }

    // Set up async receive with timeout
    auto timer = std::make_unique<boost::asio::steady_timer>(ctx);

    timer->expires_after(2s);
    timer->async_wait([callback = req->callback, eid,
                       this](const boost::system::error_code& ec) {
        if (ec != boost::asio::error::operation_aborted)
        {
            callback(-1, Response{});
            cleanupActiveRequest(eid);
            processPendingRequests(eid);
        }
    }); // Set up handler for when the timer expires

    req->timer = std::move(timer);
    activeRequests.emplace(std::move(req));
}

bool MctpRequester::enqueueRequest(mctp_eid_t eid, std::unique_ptr<Request> req)
{
    if (req->size() < sizeof(ocp::accelerator_management::BindingPciVid))
    {
        lg2::error(
            "MctpRequester: Message too small - expected={ES}, received={RS}",
            "ES", sizeof(ocp::accelerator_management::BindingPciVid), "RS",
            req->size());
        return false;
    }

    const auto* reqMsg =
        std::bit_cast<const ocp::accelerator_management::Message*>(
            req->vec.data());

    req->instanceId = reqMsg->hdr.instance_id;

    pendingRequests[eid].emplace(std::move(req));
    return true;
}

void MctpRequester::sendRecvMsg(mctp_eid_t eid, std::vector<uint8_t> reqMsg,
                                ResponseCallback callback)
{
    auto req =
        std::make_unique<Request>(std::move(reqMsg), std::move(callback));
    req->callback = std::move(callback);

    if (!enqueueRequest(eid, std::move(req)))
    {
        req->callback(-2, Response{});
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
    return (activeRequests.contains(eid));
}

void MctpRequester::cleanupActiveRequest(mctp_eid_t eid)
{
    activeRequests.erase(eid);
}

} // namespace mctp
