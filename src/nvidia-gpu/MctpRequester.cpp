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

void Requester::processRecvMsg(
    const std::span<const uint8_t> reqMsg, const std::span<uint8_t> respMsg,
    const boost::system::error_code& ec, const size_t /*length*/)
{
    uint8_t eid = recvEndPoint.eid();

    auto callbackIt = completionCallbacks.find(eid);
    if (callbackIt == completionCallbacks.end())
    {
        lg2::error(
            "MctpRequester failed to get the callback for the EID: {EID}",
            "EID", static_cast<int>(eid));
        return;
    }
    auto& callback = callbackIt->second;

    if (recvEndPoint.type() != msgType)
    {
        lg2::error("MctpRequester: Message type mismatch");
        callback(EPROTO);
        return;
    }

    expiryTimer.cancel();

    if (ec)
    {
        lg2::error(
            "MctpRequester failed to receive data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());
        callback(EIO);
        return;
    }

    if (respMsg.size() > sizeof(ocp::accelerator_management::BindingPciVid))
    {
        const auto* reqHdr =
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
            reinterpret_cast<const ocp::accelerator_management::BindingPciVid*>(
                reqMsg.data());

        uint8_t reqInstanceId = reqHdr->instance_id &
                                ocp::accelerator_management::instanceIdBitMask;
        const auto* respHdr =
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
            reinterpret_cast<const ocp::accelerator_management::BindingPciVid*>(
                respMsg.data());

        uint8_t respInstanceId = respHdr->instance_id &
                                 ocp::accelerator_management::instanceIdBitMask;

        if (reqInstanceId != respInstanceId)
        {
            lg2::error(
                "MctpRequester: Instance ID mismatch - request={REQ}, response={RESP}",
                "REQ", static_cast<int>(reqInstanceId), "RESP",
                static_cast<int>(respInstanceId));
            callback(EPROTO);
            return;
        }
    }

    callback(0);
}

void Requester::handleSendMsgCompletion(
    uint8_t eid, const std::span<const uint8_t> reqMsg,
    std::span<uint8_t> respMsg, const boost::system::error_code& ec,
    size_t /* length */)
{
    if (!completionCallbacks.contains(eid))
    {
        lg2::error(
            "MctpRequester failed to get the callback for the EID: {EID}",
            "EID", static_cast<int>(eid));
        return;
    }

    auto& callback = completionCallbacks.at(eid);

    if (ec)
    {
        lg2::error(
            "MctpRequester failed to send data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());
        callback(EIO);
        return;
    }

    expiryTimer.expires_after(2s);

    expiryTimer.async_wait([this, eid](const boost::system::error_code& ec) {
        if (ec != boost::asio::error::operation_aborted)
        {
            auto& callback = completionCallbacks.at(eid);
            callback(ETIME);
        }
    });

    mctpSocket.async_receive_from(
        boost::asio::mutable_buffer(respMsg), recvEndPoint.endpoint,
        std::bind_front(&Requester::processRecvMsg, this, reqMsg, respMsg));
}

void Requester::sendRecvMsg(uint8_t eid, const std::span<const uint8_t> reqMsg,
                            std::span<uint8_t> respMsg,
                            std::move_only_function<void(int)> callback)
{
    if (reqMsg.size() < sizeof(ocp::accelerator_management::BindingPciVid))
    {
        lg2::error("MctpRequester: Message too small");
        callback(EPROTO);
        return;
    }

    completionCallbacks[eid] = std::move(callback);

    struct sockaddr_mctp addr{};
    addr.smctp_family = AF_MCTP;
    addr.smctp_addr.s_addr = eid;
    addr.smctp_type = msgType;
    addr.smctp_tag = MCTP_TAG_OWNER;

    using endpoint = boost::asio::generic::datagram_protocol::endpoint;
    endpoint sendEndPoint{&addr, sizeof(addr)};

    mctpSocket.async_send_to(
        boost::asio::const_buffer(reqMsg), sendEndPoint,
        std::bind_front(&Requester::handleSendMsgCompletion, this, eid, reqMsg,
                        respMsg));
}

void MctpRequester::sendRecvMsg(uint8_t eid, std::span<const uint8_t> reqMsg,
                                std::span<uint8_t> respMsg,
                                std::move_only_function<void(int)> callback)
{
    RequestContext reqCtx(reqMsg, respMsg, std::move(callback));

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
        eid, reqCtx.reqMsg, reqCtx.respMsg,
        std::bind_front(&MctpRequester::handleResult, this, eid));
}

} // namespace mctp
