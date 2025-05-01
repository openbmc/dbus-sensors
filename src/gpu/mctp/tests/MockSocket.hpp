/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "types.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/system/error_code.hpp>

#include <cassert>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mctp
{

/**
 * @brief Mock implementation of MCTP socket for testing
 *
 * This class extends MctpSocket to allow for controlled testing without actual
 * socket operations by overriding socket control methods.
 */
class MockSocket
{
  public:
    /**
     * @brief Constructor
     * @param ctx - The IO context to use
     */
    explicit MockSocket(boost::asio::io_context& ctx) : ctx(ctx) {}

    bool send(eid_t eid, const Request& req)
    {
        receivedRequests[eid] = req.vec;
        return true;
    }

    void recv(const MctpSocketCallback& callback)
    {
        using namespace std::chrono_literals;

        auto index = timers.size();

        timers.push_back(std::make_unique<boost::asio::steady_timer>(ctx));
        timers[index]->expires_after(10ms);

        timers[index]->async_wait([callback, index,
                                   this](const boost::system::error_code& err) {
            if (err)
            {
                callback(err, 0, nullptr);
                return;
            }

            assert(!receivedRequests.empty());

            const eid_t eid = receivedRequests.begin()->first;
            std::vector<uint8_t> request =
                std::move(receivedRequests.begin()->second);

            receivedRequests.erase(receivedRequests.begin());

            callback(err, eid, std::make_unique<Response>(std::move(request)));
            timers[index].reset();
        });
    }

  private:
    /** @brief IO context */
    boost::asio::io_context& ctx;

    /** @brief Map of timers by endpoint ID */
    std::vector<std::unique_ptr<boost::asio::steady_timer>> timers;

    /** @brief Map of received requests */
    std::unordered_map<eid_t, std::vector<uint8_t>> receivedRequests;
};

} // namespace mctp
