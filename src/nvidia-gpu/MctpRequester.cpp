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

#include <bit>
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

static std::expected<uint8_t, std::error_code> getIid(
    std::span<const uint8_t> buffer)
{
    if (buffer.size() < sizeof(ocp::accelerator_management::BindingPciVid))
    {
        return std::unexpected(
            std::make_error_code(std::errc::invalid_argument));
    }

    const auto* hdr =
        std::bit_cast<const ocp::accelerator_management::BindingPciVid*>(
            buffer.data());
    return hdr->instance_id & ocp::accelerator_management::instanceIdBitMask;
}

MctpRequester::MctpRequester(boost::asio::io_context& ctx) :
    mctpSocket(ctx, boost::asio::generic::datagram_protocol{AF_MCTP, 0}),
    expiryTimer(ctx)
{}

void MctpRequester::bindReceive()
{
    mctpSocket.async_receive_from(
        boost::asio::buffer(buffer), recvEndPoint,
        std::bind_front(&MctpRequester::processRecvMsg, this));
}

void MctpRequester::processRecvMsg(const boost::system::error_code& ec,
                                   const size_t length)
{
    const auto* respAddr =
        std::bit_cast<const struct sockaddr_mctp*>(recvEndPoint.data());

    uint8_t eid = respAddr->smctp_addr.s_addr;

    if (respAddr->smctp_type != msgType)
    {
        // we received a message that this handler doesn't support
        // drop it on the floor and rebind receive_from
        lg2::error("MctpRequester: Message type mismatch. We received {MSG}",
                   "MSG", respAddr->smctp_type);
        bindReceive();
        return;
    }

    if (ec)
    {
        lg2::error(
            "MctpRequester failed to receive data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());
        expiryTimer.cancel();
        handleResult(eid, static_cast<std::error_code>(ec), {});
        return;
    }

    // if the received length was greater than our buffer, we would've truncated
    // and gotten an error code in asio
    std::span<const uint8_t> responseBuffer{buffer.begin(),
                                            buffer.begin() + length};

    auto iid = getIid(responseBuffer);
    if (!iid)
    {
        // we received something from the device,
        // but we aren't able to parse iid
        // report this as an error to the client
        lg2::error("MctpRequester: Unable to decode message from eid {EID}",
                   "EID", eid);
        expiryTimer.cancel();
        handleResult(eid, std::make_error_code(std::errc::message_size), {});
        return;
    }

    auto it = requestContextQueues.find(eid);
    if (it == requestContextQueues.end())
    {
        // something very bad has happened here
        // we've received a packet that is a response
        // to something we never sent out
        // do our best and rebind receive and keep the timer running
        lg2::error("Unable to match request to response");
        bindReceive();
        return;
    }

    if (*iid != it->second.iid)
    {
        // we received an iid that doesn't match the one we sent
        // rebind async_receive_from and drop this packet on the floor
        lg2::error("Invalid iid {IID} from eid {EID}", "IID", *iid, "EID", eid);
        bindReceive();
        return;
    }

    expiryTimer.cancel();
    handleResult(eid, std::error_code{}, responseBuffer);
    bindReceive();
}

void MctpRequester::handleSendMsgCompletion(
    uint8_t eid, const boost::system::error_code& ec, size_t /* length */)
{
    if (ec)
    {
        lg2::error(
            "MctpRequester failed to send data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());
        handleResult(eid, static_cast<std::error_code>(ec), {});
        return;
    }

    expiryTimer.expires_after(2s);

    expiryTimer.async_wait([this, eid](const boost::system::error_code& ec) {
        if (ec != boost::asio::error::operation_aborted)
        {
            handleResult(eid, std::make_error_code(std::errc::timed_out), {});
        }
    });

    bindReceive();
}

void MctpRequester::sendRecvMsg(
    uint8_t eid, std::span<const uint8_t> reqMsg,
    std::move_only_function<void(const std::error_code&,
                                 std::span<const uint8_t>)>
        callback)
{
    auto reqCtx = std::make_unique<RequestContext>(reqMsg, std::move(callback));

    auto& queue = requestContextQueues[eid].queue;
    queue.push_back(std::move(reqCtx));

    if (queue.size() == 1)
    {
        processQueue(eid);
    }
}

void MctpRequester::handleResult(uint8_t eid, const std::error_code& ec,
                                 std::span<const uint8_t> buffer)
{
    auto& queue = requestContextQueues[eid].queue;
    const auto& reqCtx = queue.front();

    reqCtx->callback(ec, buffer); // Call the original callback

    queue.pop_front();

    processQueue(eid);
}

std::expected<uint8_t, std::error_code> MctpRequester::getNextIid(uint8_t eid)
{
    auto it = requestContextQueues.find(eid);
    if (it == requestContextQueues.end())
    {
        return std::unexpected(
            std::make_error_code(std::errc::invalid_argument));
    }

    auto& iid = it->second.iid;
    ++iid;
    iid &= ocp::accelerator_management::instanceIdBitMask;
    return iid;
}

static std::expected<void, std::error_code> injectIid(std::span<uint8_t> buffer,
                                                      uint8_t iid)
{
    if (buffer.size() < sizeof(ocp::accelerator_management::BindingPciVid))
    {
        return std::unexpected(
            std::make_error_code(std::errc::invalid_argument));
    }

    if (iid > ocp::accelerator_management::instanceIdBitMask)
    {
        return std::unexpected(
            std::make_error_code(std::errc::invalid_argument));
    }

    auto* header = std::bit_cast<ocp::accelerator_management::BindingPciVid*>(
        buffer.data());

    header->instance_id &= ~ocp::accelerator_management::instanceIdBitMask;
    header->instance_id |= iid;
    return {};
}

void MctpRequester::processQueue(uint8_t eid)
{
    auto& queue = requestContextQueues[eid].queue;

    if (queue.empty())
    {
        return;
    }

    const auto& reqCtx = queue.front();

    auto iid = getNextIid(eid);
    if (!iid)
    {
        lg2::error("MctpRequester: Unable to get next iid");
        handleResult(eid, iid.error(), {});
        return;
    }

    auto success = injectIid(reqCtx->reqMsg, *iid);
    if (!success)
    {
        lg2::error("MctpRequester: unable to set iid");
        handleResult(eid, success.error(), {});
        return;
    }

    struct sockaddr_mctp addr{};
    addr.smctp_family = AF_MCTP;
    addr.smctp_addr.s_addr = eid;
    addr.smctp_type = msgType;
    addr.smctp_tag = MCTP_TAG_OWNER;

    sendEndPoint = {&addr, sizeof(addr)};

    mctpSocket.async_send_to(
        boost::asio::const_buffer(reqCtx->reqMsg.data(), reqCtx->reqMsg.size()),
        sendEndPoint,
        std::bind_front(&MctpRequester::handleSendMsgCompletion, this, eid));
}

} // namespace mctp
