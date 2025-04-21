/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "MctpRequester.hpp"

#include <linux/mctp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <GpuMctpVdm.hpp>
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

using namespace std::literals;

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
}

void MctpRequester::processRecvMsg(
    mctp_eid_t eid, const std::vector<uint8_t>& reqMsg,
    const std::function<void(int, std::vector<uint8_t>)>& callback,
    size_t peekedLength) const
{
    // Receive message
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
        lg2::error("MctpRequester: Failed to receive message");
        callback(-2, std::vector<uint8_t>{});
        return;
    }

    if (respAddr.smctp_type != msgType)
    {
        lg2::error("MctpRequester: Message type mismatch");
        callback(-3, std::move(fullRespMsg));
        return;
    }

    mctp_eid_t respEid = respAddr.smctp_addr.s_addr;

    if (respEid != eid)
    {
        lg2::error(
            "MctpRequester: EID mismatch - expected={EID}, received={REID}",
            "EID", eid, "REID", respEid);
        callback(-4, std::move(fullRespMsg));
        return;
    }

    if (receivedLength > sizeof(ocp::accelerator_management::BindingPciVid))
    {
        ocp::accelerator_management::BindingPciVid reqHdr{};
        std::memcpy(&reqHdr, reqMsg.data(),
                    sizeof(ocp::accelerator_management::BindingPciVid));

        ocp::accelerator_management::BindingPciVid respHdr{};
        std::memcpy(&respHdr, fullRespMsg.data(),
                    sizeof(ocp::accelerator_management::BindingPciVid));

        if (reqHdr.instance_id != respHdr.instance_id)
        {
            lg2::error(
                "MctpRequester: Instance ID mismatch - request={REQ}, response={RESP}",
                "REQ", static_cast<int>(reqHdr.instance_id), "RESP",
                static_cast<int>(respHdr.instance_id));
            callback(-5, std::move(fullRespMsg));
            return;
        }
    }

    callback(0, std::move(fullRespMsg));
}

void MctpRequester::sendRecvMsg(
    mctp_eid_t eid, const std::vector<uint8_t>& reqMsg,
    const std::function<void(int, std::vector<uint8_t>)>& callback)
{
    std::vector<uint8_t> respMsg{};

    if (reqMsg.size() < sizeof(ocp::accelerator_management::BindingPciVid))
    {
        lg2::error("MctpRequester: Message too small");
        callback(-2, respMsg);
        return;
    }

    // Create address structure
    struct sockaddr sockAddr{};
    struct sockaddr_mctp addr{};
    addr.smctp_family = AF_MCTP;
    addr.smctp_addr.s_addr = eid;
    addr.smctp_type = msgType;
    addr.smctp_tag = MCTP_TAG_OWNER;

    std::memcpy(&sockAddr, &addr, sizeof(addr));

    // Send message
    ssize_t rc = sendto(sockfd, reqMsg.data(), reqMsg.size(), 0, &sockAddr,
                        sizeof(addr));
    if (rc < 0)
    {
        lg2::error(
            "MctpRequester failed send data to the MCTP Socket - Error={EC}.",
            "EC", rc);
        callback(rc, respMsg);
        return;
    }

    // Set up async receive with timeout
    auto timer = std::make_shared<boost::asio::steady_timer>(ctx);
    timer->expires_after(2s);

    // Set up handler for when the timer expires
    timer->async_wait([callback, timer](const boost::system::error_code& ec) {
        if (ec != boost::asio::error::operation_aborted)
        {
            callback(-1, std::vector<uint8_t>{});
        }
    });

    // Set up asynchronous receive
    mctpSocket.async_receive(
        boost::asio::buffer(respMsg), MSG_PEEK | MSG_TRUNC,
        [this, eid, reqMsg, callback,
         timer](const boost::system::error_code& ec, size_t peekedLength) {
            // Cancel the timer since we got a response
            timer->cancel();

            if (ec)
            {
                lg2::error(
                    "MctpRequester failed to receive data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
                    "EC", ec.value(), "ER", ec.message());
                callback(-1, std::vector<uint8_t>{});
                return;
            }

            this->processRecvMsg(eid, reqMsg, callback, peekedLength);
        });
}

} // namespace mctp
