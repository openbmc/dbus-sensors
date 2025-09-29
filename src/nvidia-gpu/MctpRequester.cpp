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
#include <format>
#include <functional>
#include <optional>
#include <span>
#include <stdexcept>
#include <system_error>
#include <utility>

using namespace std::literals;

namespace mctp
{

static const ocp::accelerator_management::BindingPciVid* getHeaderFromBuffer(
    std::span<const uint8_t> buffer)
{
    if (buffer.size() < sizeof(ocp::accelerator_management::BindingPciVid))
    {
        return nullptr;
    }

    return std::bit_cast<const ocp::accelerator_management::BindingPciVid*>(
        buffer.data());
}

static std::optional<uint8_t> getIid(std::span<const uint8_t> buffer)
{
    const ocp::accelerator_management::BindingPciVid* header =
        getHeaderFromBuffer(buffer);
    if (header == nullptr)
    {
        return std::nullopt;
    }
    return header->instance_id & ocp::accelerator_management::instanceIdBitMask;
}

static std::optional<bool> getRequestBit(std::span<const uint8_t> buffer)
{
    const ocp::accelerator_management::BindingPciVid* header =
        getHeaderFromBuffer(buffer);
    if (header == nullptr)
    {
        return std::nullopt;
    }
    return header->instance_id & ocp::accelerator_management::requestBitMask;
}

MctpRequester::MctpRequester(boost::asio::io_context& ctx) :
    io{ctx},
    mctpSocket(ctx, boost::asio::generic::datagram_protocol{AF_MCTP, 0})
{
    startReceive();
}

void MctpRequester::startReceive()
{
    mctpSocket.async_receive_from(
        boost::asio::buffer(buffer), recvEndPoint.endpoint,
        std::bind_front(&MctpRequester::processRecvMsg, this));
}

void MctpRequester::processRecvMsg(const boost::system::error_code& ec,
                                   const size_t length)
{
    std::optional<uint8_t> expectedEid = recvEndPoint.eid();
    std::optional<uint8_t> receivedMsgType = recvEndPoint.type();

    if (!expectedEid || !receivedMsgType)
    {
        // we were handed an endpoint that can't be treated as an MCTP endpoint
        // This is probably a kernel bug...yell about it and rebind.
        lg2::error("MctpRequester: invalid endpoint");
        return;
    }

    if (*receivedMsgType != msgType)
    {
        // we received a message that this handler doesn't support
        // drop it on the floor and rebind receive_from
        lg2::error("MctpRequester: Message type mismatch. We received {MSG}",
                   "MSG", *receivedMsgType);
        return;
    }

    uint8_t eid = *expectedEid;

    if (ec)
    {
        lg2::error(
            "MctpRequester failed to receive data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());
        handleResult(eid, static_cast<std::error_code>(ec), {});
        return;
    }

    // if the received length was greater than our buffer, we would've truncated
    // and gotten an error code in asio
    std::span<const uint8_t> responseBuffer{buffer.data(), length};

    std::optional<uint8_t> optionalIid = getIid(responseBuffer);
    std::optional<bool> isRq = getRequestBit(responseBuffer);
    if (!optionalIid || !isRq)
    {
        // we received something from the device,
        // but we aren't able to parse iid byte
        // drop this packet on the floor
        // and rely on the timer to notify the client
        lg2::error("MctpRequester: Unable to decode message from eid {EID}",
                   "EID", eid);
        return;
    }

    if (isRq.value())
    {
        // we received a request from a downstream device.
        // We don't currently support this, drop the packet
        // on the floor and rebind receive, keep the timer running
        return;
    }

    uint8_t iid = *optionalIid;

    auto it = requestContextQueues.find(eid);
    if (it == requestContextQueues.end())
    {
        // something very bad has happened here
        // we've received a packet that is a response
        // from a device we've never talked to
        // do our best and rebind receive and keep the timer running
        lg2::error("Unable to match request to response");
        return;
    }

    if (iid != it->second.iid)
    {
        // we received an iid that doesn't match the one we sent
        // rebind async_receive_from and drop this packet on the floor
        lg2::error("Invalid iid {IID} from eid {EID}, expected {E_IID}", "IID",
                   iid, "EID", eid, "E_IID", it->second.iid);
        return;
    }

    handleResult(eid, std::error_code{}, responseBuffer);
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

    auto it = requestContextQueues.find(eid);
    if (it == requestContextQueues.end())
    {
        // something very bad has happened here,
        // we've sent something to a device that we have
        // no record of. yell loudly and bail
        lg2::error(
            "MctpRequester completed send for an EID that we have no record of");
        return;
    }

    boost::asio::steady_timer& expiryTimer = it->second.timer;
    expiryTimer.expires_after(2s);

    expiryTimer.async_wait([this, eid](const boost::system::error_code& ec) {
        if (ec != boost::asio::error::operation_aborted)
        {
            lg2::error("Operation timed out on eid {EID}", "EID", eid);
            handleResult(eid, std::make_error_code(std::errc::timed_out), {});
        }
    });
}

void MctpRequester::sendRecvMsg(
    uint8_t eid, std::span<const uint8_t> reqMsg,
    std::move_only_function<void(const std::error_code&,
                                 std::span<const uint8_t>)>
        callback)
{
    RequestContext reqCtx{reqMsg, std::move(callback)};

    // try_emplace only affects the result if the key does not already exist
    auto [it, inserted] = requestContextQueues.try_emplace(eid, io);
    (void)inserted;

    auto& queue = it->second.queue;
    queue.push_back(std::move(reqCtx));

    if (queue.size() == 1)
    {
        processQueue(eid);
    }
}

static bool isFatalError(const std::error_code& ec)
{
    return ec &&
           (ec != std::errc::timed_out && ec != std::errc::host_unreachable);
}

void MctpRequester::handleResult(uint8_t eid, const std::error_code& ec,
                                 std::span<const uint8_t> buffer)
{
    auto it = requestContextQueues.find(eid);
    if (it == requestContextQueues.end())
    {
        lg2::error("We tried to a handle a result for an eid we don't have");

        startReceive();
        return;
    }

    auto& queue = it->second.queue;
    auto& reqCtx = queue.front();

    it->second.timer.cancel();

    reqCtx.callback(ec, buffer); // Call the original callback

    if (isFatalError(ec))
    {
        // some errors are fatal, since these are datagrams,
        // we won't get a receive path error message.
        // and since this daemon services all nvidia iana commands
        // for a given system, we should only restart the service if its
        // unrecoverable, i.e. if we get error codes that the client
        // can't reasonably deal with. If thats the cause, restart
        // and hope that we can deal with it then.
        // since we're fully async, the only reasonable way to bubble
        // this issue up is to chuck an exception and let main deal with it.
        // alternatively we could call cancel on the io_context, but there's
        // not a great way to figure *what* happened.
        throw std::runtime_error(std::format(
            "eid {} encountered a fatal error: {}", eid, ec.message()));
    }

    startReceive();

    queue.pop_front();

    processQueue(eid);
}

std::optional<uint8_t> MctpRequester::getNextIid(uint8_t eid)
{
    auto it = requestContextQueues.find(eid);
    if (it == requestContextQueues.end())
    {
        return std::nullopt;
    }

    uint8_t& iid = it->second.iid;
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
    auto it = requestContextQueues.find(eid);
    if (it == requestContextQueues.end())
    {
        lg2::error("We are attempting to process a queue that doesn't exist");
        return;
    }

    auto& queue = it->second.queue;

    if (queue.empty())
    {
        return;
    }
    auto& reqCtx = queue.front();

    std::span<uint8_t> req{reqCtx.reqMsg.data(), reqCtx.reqMsg.size()};

    std::optional<uint8_t> iid = getNextIid(eid);
    if (!iid)
    {
        lg2::error("MctpRequester: Unable to get next iid");
        handleResult(eid, std::make_error_code(std::errc::no_such_device), {});
        return;
    }

    std::expected<void, std::error_code> success = injectIid(req, *iid);
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
    using endpoint = boost::asio::generic::datagram_protocol::endpoint;
    endpoint sendEndPoint{&addr, sizeof(addr)};

    mctpSocket.async_send_to(
        boost::asio::const_buffer(req.data(), req.size()), sendEndPoint,
        std::bind_front(&MctpRequester::handleSendMsgCompletion, this, eid));
}

} // namespace mctp
