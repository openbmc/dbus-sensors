/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/local/datagram_protocol.hpp>
#include <boost/asio/socket_base.hpp>

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
    /**
     * @brief Default constructor
     */
    MctpRequester() = delete;

    /**
     * @brief Copy constructor
     */
    MctpRequester(const MctpRequester&) = delete;

    /**
     * @brief Move constructor
     */
    MctpRequester(MctpRequester&&) = delete;

    /**
     * @brief Copy assignment operator (deleted)
     * @return Reference to this MctpRequester
     */
    MctpRequester& operator=(const MctpRequester&) = delete;

    /**
     * @brief Move assignment operator (deleted)
     * @return Reference to this MctpRequester
     */
    MctpRequester& operator=(MctpRequester&&) = delete;

    /**
     * @brief Constructor
     * @param ctx - The IO context to use
     * @param msgType - The message type to use
     * @param verbose - Whether to enable verbose logging
     */
    MctpRequester(boost::asio::io_context& ctx, uint8_t msgType,
                  bool verbose = false);

    /**
     * @brief Destructor
     *
     * Cleans up resources used by the MctpRequester instance.
     */
    ~MctpRequester();

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
    /** @brief Enable verbose logging if true */
    bool verbose;

    /** @brief IO context to use */
    boost::asio::io_context& ctx;

    /** @brief Socket file descriptor */
    int sockfd = -1;

    /** @brief Local socket */
    boost::asio::local::datagram_protocol::socket mctpSocket;

    /** @brief Message flags */
    boost::asio::socket_base::message_flags flag;

    /** @brief MCTP message type */
    uint8_t msgType;
};
} // namespace mctp
