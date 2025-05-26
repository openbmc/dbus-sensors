/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "types.hpp"

#include <linux/mctp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <boost/asio/io_context.hpp>
#include <boost/asio/local/datagram_protocol.hpp>
#include <boost/asio/socket_base.hpp>
#include <phosphor-logging/lg2.hpp>

#include <bit>
#include <cstdint>
#include <system_error>
#include <vector>

namespace mctp
{
/** @brief MCTP socket class
 *
 * This class represents an MCTP socket. It implements the Socket concept.
 *
 */
class MctpSocket
{
  public:
    /** @brief Constructor
     *
     * @param[in] ctx - The IO context to use.
     */
    explicit MctpSocket(boost::asio::io_context& ctx) :
        sockfd(socket(AF_MCTP, SOCK_DGRAM, 0)), sock(ctx), ctx(ctx)
    {
        if (sockfd < 0)
        {
            lg2::error("MctpSocket: Failed to create MCTP socket");
            return;
        }

        boost::system::error_code err;
        sock.assign(boost::asio::local::datagram_protocol{}, sockfd, err);

        if (err)
        {
            lg2::error(
                "MctpSocket: Failed to connect to the MCTP socket - ErrorCode={EC}, Error={ER}.",
                "EC", err.value(), "ER", err.message());
            return;
        }

        sock.non_blocking(true);
    }

    ~MctpSocket()
    {
        if (sockfd > 0)
        {
            close(sockfd);
        }
    }

    /** @brief Send a request to the MCTP socket
     *
     * @param[in] eid - The endpoint ID to send the request to.
     * @param[in] req - The request to send.
     * @return True if the request was sent successfully, false otherwise.
     */
    bool send(eid_t eid, const Request& req) const
    {
        struct sockaddr_mctp addr{};
        addr.smctp_family = AF_MCTP;
        addr.smctp_addr.s_addr = eid;
        addr.smctp_type = req.msgtype;
        addr.smctp_tag = MCTP_TAG_OWNER;

        // Send message
        ssize_t rc =
            sendto(sockfd, req.vec.data(), req.vec.size(), 0,
                   std::bit_cast<struct sockaddr*>(&addr), sizeof(addr));
        if (rc < 0)
        {
            lg2::error("Failed to send data to the MCTP Socket - Error={EC}.",
                       "EC", rc);
            return false;
        }
        return true;
    }

    /** @brief Receive a response from the MCTP socket
     *
     * @param[in] callback - The callback to invoke when the response is
     * received.
     */
    void recv(const MctpSocketCallback& callback)
    {
        sock.async_wait(
            boost::asio::socket_base::wait_read,
            [this, callback](const boost::system::error_code& ec) {
                if (ec)
                {
                    callback(ec, 0, {});
                    return;
                }

                struct sockaddr_mctp addr{};
                socklen_t addrlen = sizeof(addr);
                ssize_t peeklen =
                    recvfrom(sockfd, nullptr, 0, MSG_PEEK | MSG_TRUNC,
                             std::bit_cast<struct sockaddr*>(&addr), &addrlen);

                if (peeklen <= 0)
                {
                    lg2::error("Failed to peek message length - Error={EC}.",
                               "EC", peeklen);
                    callback(std::make_error_code(std::errc::io_error), 0, {});
                    return;
                }

                // Now receive the full message
                std::vector<uint8_t> resp(peeklen);
                ssize_t recvlen =
                    recvfrom(sockfd, resp.data(), peeklen, 0,
                             std::bit_cast<struct sockaddr*>(&addr), &addrlen);

                if (recvlen <= 0)
                {
                    lg2::error("Failed to receive message - Error={EC}.", "EC",
                               recvlen);
                    callback(std::make_error_code(std::errc::io_error), 0, {});
                    return;
                }

                if (peeklen != recvlen)
                {
                    lg2::error(
                        "Received message of unexpected length - expected={EL}, received={RL}",
                        "EL", peeklen, "RL", recvlen);
                    callback(std::make_error_code(std::errc::bad_message), 0,
                             {});
                    return;
                }

                const eid_t eid = addr.smctp_addr.s_addr;

                callback({}, eid, resp);
            });
    }

  protected:
    int sockfd;
    boost::asio::local::datagram_protocol::socket sock;
    boost::asio::io_context& ctx;
};
} // namespace mctp
