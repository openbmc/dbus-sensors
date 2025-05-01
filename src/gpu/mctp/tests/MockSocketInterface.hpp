/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "mctp/types.hpp"

#include <cassert>
#include <memory>

namespace mctp
{

/**
 * @brief Mock implementation of MCTP socket interface for testing
 *
 * This class provides a mock implementation of the SocketInterface concept
 * that can be used for testing. It allows setting up expected requests and
 * responses for testing without actual socket communication.
 */
class MockSocketInterface
{
  public:
    MockSocketInterface() noexcept = default;
    ~MockSocketInterface() noexcept = default;

    MockSocketInterface(const MockSocketInterface&) = delete;
    MockSocketInterface& operator=(const MockSocketInterface&) = delete;

    MockSocketInterface(MockSocketInterface&&) = delete;
    MockSocketInterface& operator=(MockSocketInterface&&) = delete;

    /**
     * @brief Initialize the socket
     * @return Weak pointer to socket fd
     */
    std::weak_ptr<SocketFd> init() noexcept
    {
        // Create a mock file descriptor (set to a positive value to indicate
        // success)
        mockFd = std::make_shared<SocketFd>(1);
        return mockFd;
    }

    /**
     * @brief Send a request
     * @param eid - The endpoint ID to send to
     * @param request - The request data
     * @return True if successful, false otherwise
     */
    bool send(eid_t eid, const Request& request) noexcept
    {
        receivedRequests[eid] = request.vec;
        return true;
    }

    /**
     * @brief Receive a response
     * @return Pair of endpoint ID and response data
     */
    std::pair<eid_t, std::unique_ptr<Response>> recv() noexcept
    {
        assert(!receivedRequests.empty());

        const eid_t eid = receivedRequests.begin()->first;
        std::vector<uint8_t> request =
            std::move(receivedRequests.begin()->second);

        receivedRequests.erase(receivedRequests.begin());

        return {eid, std::make_unique<Response>(std::move(request))};
    }

  private:
    /** @brief Mock file descriptor */
    std::shared_ptr<SocketFd> mockFd;

    /** @brief Map of received requests */
    std::unordered_map<eid_t, std::vector<uint8_t>> receivedRequests;
};

} // namespace mctp
