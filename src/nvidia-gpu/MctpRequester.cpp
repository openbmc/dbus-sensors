/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpRequester.hpp"

#include <linux/mctp.h>
#include <sys/socket.h>

#include <OcpMctpVdm.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/generic/datagram_protocol.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/devector.hpp>
#include <phosphor-logging/lg2.hpp>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <span>
#include <tuple>
#include <utility>

using namespace std::literals;

namespace mctp
{

static uint8_t getIid(const auto& buffer)
{
    const auto* respHdr =
        std::bit_cast<const ocp::accelerator_management::BindingPciVid*>(
            buffer.data());

    uint8_t instanceId = respHdr->instance_id &
                         ocp::accelerator_management::instanceIdBitMask;
    return instanceId;
}

void Requester::notifyClient(int rc)
{
    auto callback = std::move(cb);
    hasTransactionPending = false;
    callback(rc);
}

Requester::Requester(boost::asio::io_context& ctx, uint8_t eid) :
    mctpSocket(ctx, boost::asio::generic::datagram_protocol{AF_MCTP, 0}),
    expiryTimer(ctx), eid{eid}
{
    struct sockaddr_mctp addr = {};
    addr.smctp_family = AF_MCTP;
    addr.smctp_addr.s_addr = eid;
    addr.smctp_type = msgType;

    boost::asio::generic::datagram_protocol::endpoint recvEndpoint{
        &addr, sizeof(addr)};
    mctpSocket.bind(recvEndpoint);
}

void Requester::processRecvMsg(const boost::system::error_code& ec,
                               const size_t /*length*/)
{
    if (ec)
    {
        lg2::error(
            "MctpRequester failed to receive data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());
        expiryTimer.cancel();
        notifyClient(EIO);
        return;
    }

    if (respMsg.size() < sizeof(ocp::accelerator_management::BindingPciVid))
    {
        // we received a fragment of a message,
        // maybe we should rebind receive here instead
        // for now let the timeout callback fire
        lg2::error("invalid message size, unable to find iid");
        return;
    }

    uint8_t respIid = getIid(respMsg);

    if (iid != respIid)
    {
        // we received something from the endpoint that didn't match our iid
        // eventually this will be used for XID reporting,
        // which will need its own set of handling.
        // For now, we will just receive again hoping we get the right packet
        // If not, the expery timer will fire
        // in any case, drop the packet on the floor and move on
        mctpSocket.async_receive(
            respMsg, std::bind_front(&Requester::processRecvMsg, this));
        lg2::error(
            "MctpRequester: Instance ID mismatch - request={REQ}, response={RESP}, eid={EID}",
            "REQ", static_cast<int>(iid), "RESP", static_cast<int>(respIid),
            "EID", eid);
        return;
    }

    expiryTimer.cancel();
    notifyClient(0);
}

void Requester::handleSendMsgCompletion(const boost::system::error_code& ec,
                                        size_t /* length */)
{
    if (ec)
    {
        lg2::error(
            "MctpRequester failed to send data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());
        notifyClient(EIO);
        return;
    }

    expiryTimer.expires_after(2s);

    expiryTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec != boost::asio::error::operation_aborted)
        {
            lg2::error("eid {EID} timed out\n", "EID", eid);
            notifyClient(ETIME);
        }
    });

    mctpSocket.async_receive(boost::asio::mutable_buffer(respMsg),
                             std::bind_front(&Requester::processRecvMsg, this));
}

void Requester::sendRecvMsg(const std::span<const uint8_t> reqMsg,
                            std::span<uint8_t> respMsg,
                            std::move_only_function<void(int)> callback)
{
    if (reqMsg.size() < sizeof(ocp::accelerator_management::BindingPciVid))
    {
        lg2::error("MctpRequester: Message too small");
        callback(EPROTO);
        return;
    }

    uint8_t iid = getIid(reqMsg);
    if (hasTransactionPending)
    {
        lg2::error("MctpRequester: transaction already running");
        callback(EPROTO);
        return;
    }

    cb = std::move(callback);
    hasTransactionPending = true;
    this->respMsg = respMsg;
    this->iid = iid;

    struct sockaddr_mctp addr{};
    addr.smctp_family = AF_MCTP;
    addr.smctp_addr.s_addr = eid;
    addr.smctp_type = msgType;
    addr.smctp_tag = MCTP_TAG_OWNER;

    sendEndPoint = {&addr, sizeof(addr)};

    mctpSocket.async_send_to(
        boost::asio::const_buffer(reqMsg), sendEndPoint,
        std::bind_front(&Requester::handleSendMsgCompletion, this));
}

void QueuingRequester::sendRecvMsg(uint8_t eid, std::span<const uint8_t> reqMsg,
                                   std::span<uint8_t> respMsg,
                                   std::move_only_function<void(int)> callback)
{
    // ideally we could use a resonably sized circular buffer here,
    // to avoid the need for a unique pointer
    // maybe that could be a follow up commit. This is the only
    // allocation on the hot path.
    auto reqCtx =
        std::make_unique<RequestContext>(reqMsg, respMsg, std::move(callback));

    // Add request to queue
    auto it = requestContextQueues.find(eid);

    if (it == requestContextQueues.end())
    {
        // if we have not talked to this eid yet, create a queue
        // and socket
        HandlerSocket handler = {{}, std::make_unique<Requester>(io, eid)};
        std::tie(it, std::ignore) =
            requestContextQueues.insert({eid, std::move(handler)});
    }

    auto& handler = it->second;
    handler.queue.push_back(std::move(reqCtx));

    if (handler.queue.size() == 1)
    {
        processQueue(handler, eid);
    }
}

void QueuingRequester::handleResult(uint8_t eid, int result)
{
    auto& handler = requestContextQueues[eid];
    auto& queue = handler.queue;

    if (queue.empty())
    {
        lg2::error("Invalid callback!");
        return;
    }

    auto reqCtx = std::move(queue.front());

    reqCtx->callback(result); // Call the original callback

    queue.pop_front();

    processQueue(handler, eid);
}

void QueuingRequester::processQueue(HandlerSocket& handler, uint8_t eid)
{
    auto& queue = handler.queue;

    if (queue.empty())
    {
        return;
    }

    const auto& reqCtx = queue.front();

    handler.requester->sendRecvMsg(
        reqCtx->reqMsg, reqCtx->respMsg,
        std::bind_front(&QueuingRequester::handleResult, this, eid));
}

} // namespace mctp
