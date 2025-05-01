/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "types.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/local/datagram_protocol.hpp>
#include <boost/asio/socket_base.hpp>
#include <phosphor-logging/lg2.hpp>

#include <memory>
#include <utility>

namespace mctp
{
/** @brief MCTP socket class
 *
 * This class represents an MCTP socket. It implements the Socket concept.
 *
 * @tparam SockIntf - The socket interface to use.
 */
template <SocketInterface SockIntf>
class MctpSocket : public boost::asio::local::datagram_protocol::socket
{
  public:
    /** @brief Constructor
     *
     * @param[in] ctx - The IO context to use.
     */
    explicit MctpSocket(boost::asio::io_context& ctx) :
        boost::asio::local::datagram_protocol::socket(ctx), ctx(ctx)
    {
        std::weak_ptr<SocketFd> sockfd = sock.init();
        auto fd = sockfd.lock();

        if (!fd)
        {
            lg2::error("MctpSocket: Failed to create MCTP socket");
            return;
        }

        boost::system::error_code err;
        assign(boost::asio::local::datagram_protocol{}, *fd, err);

        if (err)
        {
            lg2::error(
                "MctpSocket: Failed to connect to the MCTP socket - ErrorCode={EC}, Error={ER}.",
                "EC", err.value(), "ER", err.message());
            return;
        }

        non_blocking(true);
    }

    /** @brief Send a request to the MCTP socket
     *
     * @param[in] eid - The endpoint ID to send the request to.
     * @param[in] req - The request to send.
     * @return True if the request was sent successfully, false otherwise.
     */
    bool send(eid_t eid, const Request& req)
    {
        return sock.send(eid, req);
    }

    /** @brief Receive a response from the MCTP socket
     *
     * @param[in] callback - The callback to invoke when the response is
     * received.
     */
    void recv(const MctpSocketCallback& callback)
    {
        async_wait(boost::asio::socket_base::wait_read,
                   [this, callback](const boost::system::error_code& err) {
                       if (err)
                       {
                           callback(err, 0, nullptr);
                           return;
                       }

                       std::pair<eid_t, std::unique_ptr<Response>> p =
                           sock.recv();

                       callback(err, p.first, std::move(p.second));
                   });
    }

    ~MctpSocket() = default;

  protected:
    SockIntf sock{};
    boost::asio::io_context& ctx;
};
} // namespace mctp
