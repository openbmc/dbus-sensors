/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpSocket.hpp"
#include "MockSocketInterface.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/local/datagram_protocol.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/system/error_code.hpp>

#include <vector>

namespace mctp
{

/**
 * @brief Mock implementation of MCTP socket for testing
 *
 * This class extends MctpSocket to allow for controlled testing without actual
 * socket operations by overriding socket control methods.
 */
class MockSocket : public MctpSocket<MockSocketInterface>
{
  public:
    /**
     * @brief Constructor
     * @param ctx - The IO context to use
     */
    explicit MockSocket(boost::asio::io_context& ctx) :
        MctpSocket<MockSocketInterface>(ctx), ctx(ctx)
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
    {}

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
    void recv(const MctpSocketCallback& callback)
    {
        using namespace std::chrono_literals;

        auto index = timers.size();

        timers.push_back(std::make_unique<boost::asio::steady_timer>(ctx));
        timers[index]->expires_after(10ms);

        timers[index]->async_wait(
            [callback, index, this](const boost::system::error_code& err) {
                if (err)
                {
                    callback(err, 0, nullptr);
                    return;
                }

                std::pair<eid_t, std::unique_ptr<Response>> p = sock.recv();

                callback(err, p.first, std::move(p.second));
                timers[index].reset();
            });
    }

  private:
    /** @brief IO context */
    boost::asio::io_context& ctx;

    /** @brief Map of timers by endpoint ID */
    std::vector<std::unique_ptr<boost::asio::steady_timer>> timers;
};

} // namespace mctp
