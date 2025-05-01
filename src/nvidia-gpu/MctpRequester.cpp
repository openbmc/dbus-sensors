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
#include <expected>
#include <functional>
#include <memory>
#include <span>
#include <system_error>
#include <utility>

using namespace std::literals;

namespace mctp
{

Requester::Requester(boost::asio::io_context& ctx, const uint8_t msgType,
                     ResponseHandler&& responseHandler) :
    mctpSocket(ctx, boost::asio::generic::datagram_protocol{AF_MCTP, 0}),
    responseHandler(std::move(responseHandler)), msgType(msgType)
{}

void Requester::processRecvMsg(
    [[maybe_unused]] const std::span<const uint8_t> reqMsg,
    const std::span<uint8_t> respMsg, const boost::system::error_code& ec,
    [[maybe_unused]] const size_t length)
{
    const auto* respAddr =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<const struct sockaddr_mctp*>(recvEndPoint.data());

    uint8_t eid = respAddr->smctp_addr.s_addr;

    if (respAddr->smctp_type != msgType)
    {
        lg2::error("MctpRequester: Message type mismatch");
        responseHandler(eid, respMsg, EPROTO);
        return;
    }

    if (ec)
    {
        lg2::error(
            "MctpRequester failed to receive data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());
        responseHandler(eid, respMsg, EIO);
        return;
    }

    // Call the response handler with success
    responseHandler(eid, respMsg, 0);
}

void Requester::handleSendMsgCompletion(
    uint8_t eid, const std::span<const uint8_t> reqMsg,
    std::span<uint8_t> respMsg, const boost::system::error_code& ec,
    size_t /* length */)
{
    if (ec)
    {
        lg2::error(
            "MctpRequester failed to send data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());
        responseHandler(eid, respMsg, EIO);
        return;
    }

    mctpSocket.async_receive_from(
        boost::asio::mutable_buffer(respMsg.data(), respMsg.size()),
        recvEndPoint,
        std::bind_front(&Requester::processRecvMsg, this, reqMsg, respMsg));
}

void Requester::sendRecvMsg(uint8_t eid, const std::span<const uint8_t> reqMsg,
                            std::span<uint8_t> respMsg)
{
    struct sockaddr_mctp addr{};
    addr.smctp_family = AF_MCTP;
    addr.smctp_addr.s_addr = eid;
    addr.smctp_type = msgType;
    addr.smctp_tag = MCTP_TAG_OWNER;

    sendEndPoint = {&addr, sizeof(addr)};

    mctpSocket.async_send_to(
        boost::asio::const_buffer(reqMsg.data(), reqMsg.size()), sendEndPoint,
        std::bind_front(&Requester::handleSendMsgCompletion, this, eid, reqMsg,
                        respMsg));
}

NvidiaMctpVdmRequester::NvidiaMctpVdmRequester(boost::asio::io_context& ctx) :
    ctx(ctx),
    requester(ctx, ocp::accelerator_management::messageType,
              std::bind_front(&NvidiaMctpVdmRequester::handleResponse, this))
{}

void NvidiaMctpVdmRequester::handleResponse(uint8_t eid,
                                            std::span<uint8_t> respMsg, int ec)
{
    if (ec != 0)
    {
        handleResult(eid, ec);
        return;
    }

    // Validate message size
    if (respMsg.size() < sizeof(ocp::accelerator_management::BindingPciVid))
    {
        lg2::error("Response size too small: {SIZE}", "SIZE", respMsg.size());
        handleResult(eid, EMSGSIZE);
        return;
    }

    auto& queue = requestContextQueues[eid];
    const auto& reqCtx = queue.front();

    const auto* reqHdr =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<const ocp::accelerator_management::BindingPciVid*>(
            reqCtx->reqMsg.data());

    uint8_t reqInstanceId =
        reqHdr->instance_id & ocp::accelerator_management::instanceIdBitMask;
    const auto* respHdr =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<const ocp::accelerator_management::BindingPciVid*>(
            respMsg.data());

    uint8_t respInstanceId =
        respHdr->instance_id & ocp::accelerator_management::instanceIdBitMask;

    if (reqInstanceId != respInstanceId)
    {
        lg2::error(
            "MctpRequester: Instance ID mismatch - request={REQ}, response={RESP}",
            "REQ", static_cast<int>(reqInstanceId), "RESP",
            static_cast<int>(respInstanceId));
        // Don't invoke the callback with error. This could be a response
        // for a previously timed out request and the response for 'this'
        // request could be in-flight
        return;
    }

    expiryTimers[eid]->cancel();

    handleResult(eid, 0);
}

void NvidiaMctpVdmRequester::sendRecvMsg(
    uint8_t eid, std::span<uint8_t> reqMsg, std::span<uint8_t> respMsg,
    std::move_only_function<void(int)> callback)
{
    if (reqMsg.size() < sizeof(ocp::accelerator_management::BindingPciVid))
    {
        lg2::error("MctpRequester: Message too small");
        callback(EMSGSIZE);
        return;
    }

    auto reqCtx =
        std::make_unique<RequestContext>(reqMsg, respMsg, std::move(callback));

    // Add request to queue
    auto& queue = requestContextQueues[eid];
    queue.push_back(std::move(reqCtx));

    if (queue.size() == 1)
    {
        processQueue(eid);
    }
}

void NvidiaMctpVdmRequester::handleResult(uint8_t eid, int result)
{
    auto& queue = requestContextQueues[eid];
    const auto& reqCtx = queue.front();

    uint8_t id = reqCtx->getInstanceId();
    instanceIdDb.free(eid, id);

    reqCtx->callback(result); // Call the original callback

    queue.pop_front();

    processQueue(eid);
}

void NvidiaMctpVdmRequester::processQueue(uint8_t eid)
{
    auto& queue = requestContextQueues[eid];

    if (queue.empty())
    {
        return;
    }

    const std::expected<uint8_t, std::error_code> id = instanceIdDb.next(eid);
    if (!id.has_value())
    {
        lg2::error("Error while allocating instance id for EID: {EID}, {ERR}",
                   "EID", eid, "ERR", id.error().message());
        return;
    }

    const auto& reqCtx = queue.front();

    // Set up expiry timer
    if (expiryTimers[eid] == nullptr)
    {
        expiryTimers[eid] = std::make_unique<boost::asio::steady_timer>(ctx);
    }

    reqCtx->setRequestInstanceId(id.value());

    requester.sendRecvMsg(eid, reqCtx->reqMsg, reqCtx->respMsg);

    expiryTimers[eid]->expires_after(2s);
    expiryTimers[eid]->async_wait(
        [this, eid](const boost::system::error_code& ec) {
            if (ec != boost::asio::error::operation_aborted)
            {
                handleResult(eid, ETIME);
            }
        });
}

} // namespace mctp
