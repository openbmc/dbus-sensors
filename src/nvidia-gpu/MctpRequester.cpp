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
#include <utility>

using namespace std::literals;

namespace mctp
{

Requester::Requester(boost::asio::io_context& ctx) :
    mctpSocket(ctx, boost::asio::generic::datagram_protocol{AF_MCTP, 0}),
    expiryTimer(ctx)
{}

void Requester::processRecvMsg(const boost::system::error_code& ec,
                               const size_t length)
{
    expiryTimer.cancel();

    if (ec)
    {
        lg2::error(
            "MctpRequester failed to receive data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());
        return;
    }

    std::span<const uint8_t> respMsg(responseBuffer.data(), length);

    uint8_t eid = recvEndPoint.eid();

    if (recvEndPoint.type() != ocp::accelerator_management::messageType)
    {
        lg2::error("MctpRequester: Message type mismatch");
        return;
    }

    if (respMsg.size() < sizeof(ocp::accelerator_management::BindingPciVid))
    {
        lg2::error("MctpRequester: Message too small to be PCI vdm");
        return;
    }

    const ocp::accelerator_management::BindingPciVid* respHdr =
        std::bit_cast<const ocp::accelerator_management::BindingPciVid*>(
            respMsg.data());

    uint8_t respInstanceId =
        respHdr->instance_id & ocp::accelerator_management::instanceIdBitMask;

    auto callbackIt =
        completionCallbacks.find(CompletionMatch(eid, respInstanceId));
    if (callbackIt == completionCallbacks.end())
    {
        lg2::error(
            "MctpRequester: Couldn't find callback for message with EID={EID} and Instance ID={INSTANCE_ID}",
            "EID", eid, "INSTANCE_ID", respInstanceId);
        return;
    }
    auto& callback = callbackIt->second;

    callback(0);
}

void Requester::handleSendMsgCompletion(
    uint8_t eid, const boost::system::error_code& ec, size_t /* length */)
{
    if (ec)
    {
        lg2::error(
            "MctpRequester failed to send data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());
        return;
    }

    if (ec)
    {
        lg2::error(
            "MctpRequester failed to send data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());
        return;
    }

    expiryTimer.expires_after(2s);

    expiryTimer.async_wait([this, eid](const boost::system::error_code& ec) {
        if (ec != boost::asio::error::operation_aborted)
        {
            // Success, do nothing
            return;
        }
        lg2::error(
            "MctpRequester: Message timed out - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());

        // TODO handle error
    });

    mctpSocket.async_receive_from(
        boost::asio::mutable_buffer(responseBuffer.data(),
                                    responseBuffer.size()),
        recvEndPoint.endpoint,
        std::bind_front(&Requester::processRecvMsg, this));
}

uint8_t Requester::getNextInstanceId()
{
    static uint8_t instanceId = 0;
    instanceId++;
    if (instanceId > ocp::accelerator_management::instanceIdBitMask)
    {
        instanceId = 0;
    }
    return instanceId;
}

void Requester::sendRecvMsg(uint8_t eid, const std::span<const uint8_t> reqMsg,
                            std::move_only_function<void(int)> callback)
{
    if (reqMsg.size() < sizeof(ocp::accelerator_management::BindingPciVid))
    {
        lg2::error("MctpRequester: Message too small");
        callback(EPROTO);
        return;
    }

    uint8_t instanceId = getNextInstanceId();

    completionCallbacks[CompletionMatch(eid, instanceId)] = std::move(callback);

    struct sockaddr_mctp addr{};
    addr.smctp_family = AF_MCTP;
    addr.smctp_addr.s_addr = eid;
    addr.smctp_type = ocp::accelerator_management::messageType;
    addr.smctp_tag = MCTP_TAG_OWNER;

    using endpoint = boost::asio::generic::datagram_protocol::endpoint;
    endpoint sendEndPoint{&addr, sizeof(addr)};

    mctpSocket.async_send_to(
        boost::asio::const_buffer(reqMsg), sendEndPoint,
        std::bind_front(&Requester::handleSendMsgCompletion, this, eid));
}

void MctpRequester::sendRecvMsg(uint8_t eid, std::span<const uint8_t> reqMsg,
                                std::move_only_function<void(int)> callback)
{
    RequestContext reqCtx(reqMsg, std::move(callback));

    // Add request to queue
    auto& queue = requestContextQueues[eid];
    queue.push_back(std::move(reqCtx));

    if (queue.size() == 1)
    {
        processQueue(eid);
    }
}

void MctpRequester::handleResult(uint8_t eid, int result)
{
    auto it = requestContextQueues.find(eid);
    if (it == requestContextQueues.end())
    {
        lg2::error("MctpRequester: Failed to find the queue for the EID: {EID}",
                   "EID", static_cast<int>(eid));
        return;
    }

    boost::container::devector<RequestContext>& queue = it->second;

    RequestContext& reqCtx = queue.front();

    // Call the original callback
    reqCtx.callback(result);

    queue.pop_front();

    processQueue(eid);
}

void MctpRequester::processQueue(uint8_t eid)
{
    auto& queue = requestContextQueues[eid];

    if (queue.empty())
    {
        return;
    }

    const auto& reqCtx = queue.front();

    requester.sendRecvMsg(
        eid, reqCtx.reqMsg,
        std::bind_front(&MctpRequester::handleResult, this, eid));
}

} // namespace mctp
