/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/local/datagram_protocol.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

// Define MCTP EID type
using mctp_eid_t = uint8_t;

namespace mctp
{
/**
 * @brief MCTP requester class
 *
 * This class provides a simple interface for sending and receiving MCTP
 * messages.
 */
class MctpRequester
{
  public:
    MctpRequester() = delete;

    MctpRequester(const MctpRequester&) = delete;

    MctpRequester(MctpRequester&&) = delete;

    MctpRequester& operator=(const MctpRequester&) = delete;

    MctpRequester& operator=(MctpRequester&&) = delete;

    /**
     * @brief Constructor
     * @param ctx - The IO context to use
     * @param msgType - The message type to use
     */
    MctpRequester(boost::asio::io_context& ctx, uint8_t msgType);

    /**
     * @brief Send an MCTP request message and receive the response
     *
     * This function sends a request message to the specified endpoint ID and
     * asynchronously waits for a response. It uses the MCTP socket to handle
     * the communication. Results are provided via the callback.
     *
     * @param[in] eid - The endpoint ID to send the message to
     * @param[in] reqMsg - The request message to send
     * @param[in] callback - Callback function to be invoked when response is
     * received The callback takes two parameters:
     *            - An integer status code (0 for success, negative for error)
     *            - A vector containing the response message bytes
     */
    void sendRecvMsg(
        mctp_eid_t eid, const std::vector<uint8_t>& reqMsg,
        const std::function<void(int, std::vector<uint8_t>)>& callback);

  private:
    /**
     * @brief Process received message
     *
     * This function processes a received message and invokes the callback with
     * the appropriate status code and response message bytes.
     *
     * @param[in] eid - The endpoint ID from which the message was received
     * @param[in] reqMsg - The received request message bytes
     * @param[in] callback - The callback function to invoke with the result
     * @param[in] peekedLength - The length of the peeked data
     */
    void processRecvMsg(
        mctp_eid_t eid, const std::vector<uint8_t>& reqMsg,
        const std::function<void(int, std::vector<uint8_t>)>& callback,
        size_t peekedLength) const;

    /** @brief IO context to use */
    boost::asio::io_context& ctx;

    /** @brief Socket file descriptor */
    int sockfd = -1;

    /** @brief Local socket */
    boost::asio::local::datagram_protocol::socket mctpSocket;

    /** @brief MCTP message type */
    uint8_t msgType;
};
} // namespace mctp
