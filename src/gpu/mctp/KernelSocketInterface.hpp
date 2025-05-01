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

#include <cstring>
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
    KernelSocketInterface() = default;
    ~KernelSocketInterface()
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

    /**
     * @brief Initialize the socket
     * @return File descriptor of the socket
     *
     * @note The fd is owned by the class and is not to be closed by the caller.
     */
    std::weak_ptr<SocketFd> init()
    {
        const SocketFd fd = socket(AF_MCTP, SOCK_DGRAM, 0);

        sockfd = fd < 0 ? nullptr : std::make_shared<SocketFd>(fd);

        return sockfd;
    }

    /**
     * @brief Send a request
     * @param eid - The endpoint ID to send to
     * @param request - The request data
     * @return True if successful, false otherwise
     */
    bool send(eid_t eid, const Request& req) const
    {
        struct sockaddr_mctp addr{};
        addr.smctp_family = AF_MCTP;
        addr.smctp_addr.s_addr = eid;
        addr.smctp_type = req.msgType;
        addr.smctp_tag = MCTP_TAG_OWNER;

        // Create address structure
        struct sockaddr sockAddr{};
        std::memcpy(&sockAddr, &addr, sizeof(addr));

        // Send message
        ssize_t rc = sendto(*sockfd, req.vec.data(), req.vec.size(), 0,
                            &sockAddr, sizeof(addr));
        if (rc < 0)
        {
            lg2::error("Failed to send data to the MCTP Socket - Error={EC}.",
                       "EC", rc);
            return false;
        }
        return true;
    }

    /**
     * @brief Receive a response
     * @return Pair of endpoint ID and response data
     */
    std::pair<eid_t, std::unique_ptr<Response>> recv() const
    {
        struct sockaddr sockAddr{};
        struct sockaddr_mctp respAddr{};
        socklen_t addrlen = sizeof(respAddr);

        ssize_t peekedLength = recvfrom(
            *sockfd, nullptr, 0, MSG_PEEK | MSG_TRUNC, &sockAddr, &addrlen);

        if (peekedLength <= 0)
        {
            lg2::error("Failed to peek message length - Error={EC}.", "EC",
                       peekedLength);
            return {0, nullptr};
        }

        // Now receive the full message
        Response resp(peekedLength);
        ssize_t receivedLength = recvfrom(*sockfd, resp.data(), peekedLength, 0,
                                          &sockAddr, &addrlen);

        if (receivedLength <= 0)
        {
            lg2::error("Failed to receive message - Error={EC}.", "EC",
                       receivedLength);
            return {0, nullptr};
        }

        if (peekedLength != receivedLength)
        {
            lg2::error(
                "Received message of unexpected length - expected={EL}, received={RL}",
                "EL", peekedLength, "RL", receivedLength);
            return {0, nullptr};
        }

        std::memcpy(&respAddr, &sockAddr, sizeof(respAddr));
        const eid_t eid = respAddr.smctp_addr.s_addr;

        return {eid, std::make_unique<Response>(std::move(resp))};
    }

  private:
    /** @brief Socket file descriptor */
    std::shared_ptr<SocketFd> sockfd;
};

} // namespace mctp
