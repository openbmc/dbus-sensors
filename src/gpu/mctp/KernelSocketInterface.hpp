/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "types.hpp"

#include <linux/mctp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <phosphor-logging/lg2.hpp>

#include <bit>
#include <memory>
#include <utility>

namespace mctp
{

/**
 * @brief Implementation of MCTP socket interface using Linux kernel socket
 *
 * This class provides an implementation of the MctpSocketInterface concept
 * using Linux kernel socket interface.
 */
class KernelSocketInterface
{
  public:
    KernelSocketInterface() noexcept = default;
    ~KernelSocketInterface() noexcept
    {
        if (sockfd)
        {
            close(*sockfd);
        }
    }

    KernelSocketInterface(const KernelSocketInterface&) = delete;
    KernelSocketInterface& operator=(const KernelSocketInterface&) = delete;

    KernelSocketInterface(KernelSocketInterface&&) = delete;
    KernelSocketInterface& operator=(KernelSocketInterface&&) = delete;

    /** @brief Initialize the socket
     *
     * @return Weak pointer to the socket file descriptor.
     */
    std::weak_ptr<SocketFd> init() noexcept
    {
        const SocketFd fd = socket(AF_MCTP, SOCK_DGRAM, 0);

        sockfd = fd < 0 ? nullptr : std::make_shared<SocketFd>(fd);

        return sockfd;
    }

    /** @brief Send a request
     *
     * @param[in] eid - The endpoint ID to send to.
     * @param[in] req - The request data.
     * @return True if successful, false otherwise.
     */
    bool send(eid_t eid, const Request& req) const noexcept
    {
        struct sockaddr_mctp addr{};
        addr.smctp_family = AF_MCTP;
        addr.smctp_addr.s_addr = eid;
        addr.smctp_type = req.msgType;
        addr.smctp_tag = MCTP_TAG_OWNER;

        // Send message
        ssize_t rc =
            sendto(*sockfd, req.vec.data(), req.vec.size(), 0,
                   std::bit_cast<struct sockaddr*>(&addr), sizeof(addr));
        if (rc < 0)
        {
            lg2::error("Failed to send data to the MCTP Socket - Error={EC}.",
                       "EC", rc);
            return false;
        }
        return true;
    }

    /** @brief Receive a response
     *
     * @return Pair of endpoint ID and response data.
     */
    std::pair<eid_t, std::unique_ptr<Response>> recv() const noexcept
    {
        struct sockaddr_mctp addr{};
        socklen_t addrlen = sizeof(addr);

        ssize_t peeklen =
            recvfrom(*sockfd, nullptr, 0, MSG_PEEK | MSG_TRUNC,
                     std::bit_cast<struct sockaddr*>(&addr), &addrlen);

        if (peeklen <= 0)
        {
            lg2::error("Failed to peek message length - Error={EC}.", "EC",
                       peeklen);
            return {0, nullptr};
        }

        // Now receive the full message
        Response resp(peeklen);
        ssize_t recvlen =
            recvfrom(*sockfd, resp.data(), peeklen, 0,
                     std::bit_cast<struct sockaddr*>(&addr), &addrlen);

        if (recvlen <= 0)
        {
            lg2::error("Failed to receive message - Error={EC}.", "EC",
                       recvlen);
            return {0, nullptr};
        }

        if (peeklen != recvlen)
        {
            lg2::error(
                "Received message of unexpected length - expected={EL}, received={RL}",
                "EL", peeklen, "RL", recvlen);
            return {0, nullptr};
        }

        const eid_t eid = addr.smctp_addr.s_addr;

        return {eid, std::make_unique<Response>(std::move(resp))};
    }

  private:
    /** @brief Socket file descriptor.
     *
     * @note The fd is owned by the class.
     */
    std::shared_ptr<SocketFd> sockfd;
};

} // namespace mctp
