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
#include <phosphor-logging/lg2.hpp>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <span>

using namespace std::literals;

namespace mctp
{

MctpRequester::MctpRequester(boost::asio::io_context& ctx) :
    ctx(ctx),
    mctpSocket(ctx, boost::asio::generic::datagram_protocol{AF_MCTP, 0})
{}

void MctpRequester::processRecvMsg(
    uint8_t eid, const std::span<const uint8_t> reqMsg,
    const std::span<uint8_t> respMsg,
    const std::shared_ptr<boost::asio::steady_timer>& timer,
    const std::shared_ptr<boost::asio::generic::datagram_protocol::endpoint>&
        ep,
    const std::function<void(int)>& callback,
    const boost::system::error_code& ec, const size_t /*length*/)
{
    // Cancel the timer since we got a response
    timer->cancel();

    if (ec)
    {
        lg2::error(
            "MctpRequester failed to receive data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());
        callback(EIO);
        return;
    }

    struct sockaddr_mctp respAddr{};
    std::memcpy(&respAddr, ep->data(), ep->size());

    if (respAddr.smctp_type != msgType)
    {
        lg2::error("MctpRequester: Message type mismatch");
        callback(EPROTO);
        return;
    }

    uint8_t respEid = respAddr.smctp_addr.s_addr;

    if (respEid != eid)
    {
        lg2::error(
            "MctpRequester: EID mismatch - expected={EID}, received={REID}",
            "EID", eid, "REID", respEid);
        callback(EPROTO);
        return;
    }

    if (respMsg.size() > sizeof(ocp::accelerator_management::BindingPciVid))
    {
        ocp::accelerator_management::BindingPciVid reqHdr{};
        std::memcpy(&reqHdr, reqMsg.data(), sizeof(reqHdr));

        uint8_t reqInstanceId =
            reqHdr.instance_id & ocp::accelerator_management::instanceIdBitMask;

        ocp::accelerator_management::BindingPciVid respHdr{};
        std::memcpy(&respHdr, respMsg.data(), sizeof(respHdr));

        uint8_t respInstanceId = respHdr.instance_id &
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

void MctpRequester::handleSendMsgCompletion(
    uint8_t eid, const std::span<const uint8_t> reqMsg,
    std::span<uint8_t> respMsg, const std::function<void(int)>& callback,
    const boost::system::error_code& ec, size_t /* length */)
{
    if (ec)
    {
        lg2::error(
            "MctpRequester failed to send data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());
        callback(EIO);
        return;
    }

    // Set up async receive with timeout
    auto timer = std::make_shared<boost::asio::steady_timer>(ctx);
    timer->expires_after(2s);

    // Set up handler for when the timer expires
    timer->async_wait([callback, timer](const boost::system::error_code& ec) {
        if (ec != boost::asio::error::operation_aborted)
        {
            callback(ETIME);
        }
    });

    auto ep =
        std::make_shared<boost::asio::generic::datagram_protocol::endpoint>();

    mctpSocket.async_receive_from(
        boost::asio::mutable_buffer(respMsg), *ep,
        std::bind_front(&MctpRequester::processRecvMsg, eid, reqMsg, respMsg,
                        timer, ep, callback));
}

void MctpRequester::sendRecvMsg(
    uint8_t eid, const std::span<const uint8_t> reqMsg,
    std::span<uint8_t> respMsg, const std::function<void(int)>& callback)
{
    if (reqMsg.size() < sizeof(ocp::accelerator_management::BindingPciVid))
    {
        lg2::error("MctpRequester: Message too small");
        callback(EPROTO);
        return;
    }

    // Create address structure
    struct sockaddr_mctp addr{};
    addr.smctp_family = AF_MCTP;
    addr.smctp_addr.s_addr = eid;
    addr.smctp_type = msgType;
    addr.smctp_tag = MCTP_TAG_OWNER;

    boost::asio::generic::datagram_protocol::endpoint ep(&addr, sizeof(addr));

    mctpSocket.async_send_to(
        boost::asio::const_buffer(reqMsg), ep,
        std::bind_front(&MctpRequester::handleSendMsgCompletion, this, eid,
                        reqMsg, respMsg, callback));
}
} // namespace mctp
