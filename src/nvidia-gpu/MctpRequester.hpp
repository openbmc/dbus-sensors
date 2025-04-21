/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <OcpMctpVdm.hpp>
#include <boost/asio/generic/datagram_protocol.hpp>
#include <boost/asio/io_context.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>

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
     */
    explicit MctpRequester(boost::asio::io_context& ctx);

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
    void sendRecvMsg(uint8_t eid, std::span<const uint8_t> reqMsg,
                     std::span<uint8_t> respMsg,
                     const std::function<void(int)>& callback);

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
    static void processRecvMsg(
        uint8_t eid, std::span<const uint8_t> reqMsg,
        std::span<uint8_t> respMsg,
        const boost::asio::generic::datagram_protocol::endpoint& ep,
        const std::function<void(int)>& callback);

    void handleSendMsgCompletion(uint8_t eid, std::span<const uint8_t> reqMsg,
                                 std::span<uint8_t> respMsg,
                                 const std::function<void(int)>& callback);

    /** @brief IO context to use */
    boost::asio::io_context& ctx;

    /** @brief Local socket */
    boost::asio::generic::datagram_protocol::socket mctpSocket;

    static constexpr size_t maxMessageSize = 65536 + 256;

    /** @brief MCTP message type */
    static constexpr uint8_t msgType = ocp::accelerator_management::messageType;
};
} // namespace mctp
