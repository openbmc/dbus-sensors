/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/local/datagram_protocol.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/system/error_code.hpp>

#include <vector>

namespace mctp
{

/**
 * @brief Mock implementation of boost::asio socket for testing
 *
 * This class extends the boost::asio::local::datagram_protocol::socket to allow
 * for controlled testing without actual socket operations.
 */
class MockSocket : public boost::asio::local::datagram_protocol::socket
{
  public:
    using BaseSocket = boost::asio::local::datagram_protocol::socket;

    /**
     * @brief Constructor
     * @param ctx - The IO context to use
     */
    explicit MockSocket(boost::asio::io_context& ctx) :
        BaseSocket(ctx), ctx(ctx)
    {}

    /**
     * @brief Override to control assign behavior
     * @param protocol - The socket protocol
     * @param fd - The file descriptor
     * @param ec - Error code
     */
    void assign([[maybe_unused]] boost::asio::local::datagram_protocol endpoint,
                [[maybe_unused]] int fd,
                [[maybe_unused]] boost::system::error_code& ec)
    {
        sockfd = fd;
    }

    /**
     * @brief Override to control non_blocking behavior
     * @param mode - True for non-blocking, false for blocking
     * @param ec - Error code
     */
    void non_blocking([[maybe_unused]] bool mode,
                      [[maybe_unused]] boost::system::error_code& ec)
    {}

    /**
     * @brief Override to control non_blocking behavior
     * @param mode - True for non-blocking, false for blocking
     */
    void non_blocking([[maybe_unused]] bool mode) {}

    /**
     * @brief Override to control async_wait behavior
     * @param condition - The condition to wait for
     * @param handler - The handler to call when ready
     */
    template <typename WaitCondition, typename WaitCompHandler>
    void async_wait([[maybe_unused]] WaitCondition condition,
                    [[maybe_unused]] WaitCompHandler handler)
    {
        using namespace std::chrono_literals;

        timers.push_back(std::make_unique<boost::asio::steady_timer>(ctx));
        timers.back()->expires_after(500ms);
        timers.back()->async_wait([handler, index = timers.size() - 1,
                                   this](const boost::system::error_code& ec) {
            handler(ec);
            timers.erase(timers.begin() + index);
        });
    }

  private:
    /** @brief IO context */
    boost::asio::io_context& ctx;

    int sockfd{};

    /** @brief Map of timers by endpoint ID */
    std::vector<std::unique_ptr<boost::asio::steady_timer>> timers;
};

} // namespace mctp
