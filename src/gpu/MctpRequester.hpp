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
#include <utility>
#include <vector>

namespace mctp
{

/** @brief MCTP endpoint ID type */
using eid_t = uint8_t;

/** @brief MCTP response */
using Response = std::vector<uint8_t>;

/** @brief MCTP response callback */
using ResponseCallback = std::function<void(int, const Response&)>;

/** @brief MCTP request */
struct Request
{
  public:
    /** @brief Constructor
     *
     * @param[in] data - The request data
     * @param[in] callback - The callback to invoke when the response is
     * received
     */
    explicit Request(std::vector<uint8_t>&& data,
                     ResponseCallback&& callback) noexcept :
        callback(std::move(callback)), vec(std::move(data)) {};

    Request() = delete;
    Request(const Request& r) = delete;

    Request(Request&& r) noexcept = default;
    Request& operator=(Request&& r) noexcept = default;
    ~Request() noexcept = default;

    /** @brief Get the size of the request
     *
     * @return The size of the request
     */
    std::size_t size() const noexcept
    {
        return vec.size();
    }

    /** @brief Instance ID */
    uint8_t instanceId{};

    /** @brief Response callback */
    ResponseCallback callback;

    /** @brief Timer for request timeout */
    std::unique_ptr<boost::asio::steady_timer> timer;

    /** @brief Request data */
    std::vector<uint8_t> vec;
};

/** @brief Queue of requests */
using RequestQueue = std::queue<std::unique_ptr<Request>>;

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
    void sendRecvMsg(eid_t eid, std::vector<uint8_t> reqMsg,
                     ResponseCallback callback);

  private:
    /**
     * @brief Await response
     *
     * This function waits for any mctp response to any sent request and
     * invokes the callback with the appropriate status code and response
     * message bytes.
     */
    void awaitResponse();

    /** @brief Enqueue a request
     *
     * @param[in] eid - The endpoint ID to send the message to
     * @param[in] reqMsg - The request message to send
     * @param[in] callback - Callback function to be invoked when response is
     * received The callback takes two parameters:
     *            - An integer status code (0 for success, negative for error)
     *            - A vector containing the response message bytes
     */
    bool enqueueRequest(eid_t eid, std::unique_ptr<Request> reqMsg);

    /** @brief Process pending requests
     *
     * This function processes pending requests for a given endpoint ID. It
     * retrieves the next request from the queue and sends it to the endpoint.
     *
     * @param[in] eid - The endpoint ID to process requests for
     */
    void processPendingRequests(eid_t eid);

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
    bool hasPendingRequests(eid_t eid) const;

    /** @brief Helper function to check if there is an active request
     *
     * This function checks if there is an active request for a given endpoint
     * ID.
     *
     * @param[in] eid - The endpoint ID to check for active request
     * @return True if there is an active request, false otherwise
     */
    bool hasActiveRequest(eid_t eid) const;

    /** @brief Cancel an active request
     *
     * This function cancels an active request for a given endpoint ID.
     *
     * @param[in] eid - The endpoint ID to cancel the request for
     */
    void cleanupActiveRequest(eid_t eid);

    /** @brief Handle a response
     *
     * This function handles a response for a given endpoint ID.
     *
     * @param[in] eid - The endpoint ID to handle the response for
     * @param[in] response - The response to handle
     */
    inline void handleResponse(eid_t eid, const Response& response);

    /** @brief IO context to use */
    boost::asio::io_context& ctx;

    /** @brief Socket file descriptor */
    int sockfd = -1;

    /** @brief Local socket */
    boost::asio::local::datagram_protocol::socket mctpSocket;

    /** @brief MCTP message type */
    uint8_t msgType;

    /** @brief Pending requests */
    std::unordered_map<eid_t, RequestQueue> pendingRequests;

    /** @brief Active request callbacks */
    std::unordered_map<eid_t, std::unique_ptr<Request>> activeRequests;
};
} // namespace mctp
