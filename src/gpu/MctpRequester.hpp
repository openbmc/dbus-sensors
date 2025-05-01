/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/local/datagram_protocol.hpp>
#include <boost/asio/steady_timer.hpp>

#include <cstdint>
#include <functional>
#include <queue>
#include <vector>

namespace mctp
{

/** @brief MCTP EID type */
using mctp_eid_t = uint8_t;

/** @brief Request */
using Request = std::vector<uint8_t>;

/** @brief Response */
using Response = std::vector<uint8_t>;

/** @brief Response callback */
using ResponseCallback = std::function<void(int, const Response&)>;

/** @brief Request queue */
using RequestQueue = std::queue<std::pair<Request, ResponseCallback>>;

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
    void sendRecvMsg(mctp_eid_t eid, Request reqMsg, ResponseCallback callback);

  private:
    /**
     * @brief Poll for received messages
     *
     * This function polls for received messages and invokes the callback with
     * the appropriate status code and response message bytes.
     */
    void asyncRecv();

    /** @brief Enqueue a request
     *
     * @param[in] eid - The endpoint ID to send the message to
     * @param[in] reqMsg - The request message to send
     * @param[in] callback - Callback function to be invoked when response is
     * received The callback takes two parameters:
     *            - An integer status code (0 for success, negative for error)
     *            - A vector containing the response message bytes
     */
    bool enqueueRequest(mctp_eid_t eid, Request&& reqMsg,
                        ResponseCallback&& callback);

    /** @brief Process pending requests
     *
     * This function processes pending requests for a given endpoint ID. It
     * retrieves the next request from the queue and sends it to the endpoint.
     *
     * @param[in] eid - The endpoint ID to process requests for
     */
    void processPendingRequests(mctp_eid_t eid);

    /** @brief Process pending requests for all endpoints
     *
     * This function processes pending requests for all endpoints.
     */
    void processPendingRequests();

    /** @brief Helper function to check if there are pending requests
     *
     * This function checks if there are pending requests for a given endpoint
     * ID.
     *
     * @param[in] eid - The endpoint ID to check for pending requests
     * @return True if there are pending requests, false otherwise
     */
    bool hasPendingRequests(mctp_eid_t eid) const;

    /** @brief Helper function to check if there is an active request
     *
     * This function checks if there is an active request for a given endpoint
     * ID.
     *
     * @param[in] eid - The endpoint ID to check for active request
     * @return True if there is an active request, false otherwise
     */
    bool hasActiveRequest(mctp_eid_t eid) const;

    /** @brief Cancel an active request
     *
     * This function cancels an active request for a given endpoint ID.
     *
     * @param[in] eid - The endpoint ID to cancel the request for
     */
    void cleanupActiveRequest(mctp_eid_t eid);

    /** @brief IO context to use */
    boost::asio::io_context& ctx;

    /** @brief Socket file descriptor */
    int sockfd = -1;

    /** @brief Local socket */
    boost::asio::local::datagram_protocol::socket mctpSocket;

    /** @brief MCTP message type */
    uint8_t msgType;

    /** @brief Pending requests */
    std::unordered_map<mctp_eid_t, RequestQueue> pendingRequests;

    /** @brief Active request callbacks */
    std::unordered_map<mctp_eid_t, std::pair<uint8_t, ResponseCallback>>
        activeCallbacks;

    /** @brief Timer */
    std::unordered_map<mctp_eid_t, std::shared_ptr<boost::asio::steady_timer>>
        activeTimers;
};
} // namespace mctp
