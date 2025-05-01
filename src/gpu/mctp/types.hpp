/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "OcpMctpVdm.hpp"

#include <boost/asio/steady_timer.hpp>
#include <phosphor-logging/lg2.hpp>

#include <bit>
#include <concepts>
#include <cstdint>
#include <functional>
#include <memory>
#include <system_error>
#include <utility>
#include <vector>

namespace mctp
{
/** @brief MCTP endpoint ID type */
using eid_t = uint8_t;

/** @brief MCTP response type
 *
 * Represents the response data from an MCTP endpoint.
 * This is a vector of bytes containing the raw response data.
 */
using Response = std::vector<uint8_t>;

/** @brief Error code type
 *
 * Represents error codes that can be returned from MCTP operations.
 */
using ErrorCode = int;

/** @brief Socket file descriptor type
 *
 * Represents a file descriptor for an MCTP socket.
 */
using SocketFd = int;

/** @brief MCTP response callback type
 *
 * Function type for handling MCTP responses.
 * @param errorCode - The error code indicating success or failure
 * @param response - The response data received from the MCTP endpoint
 */
using ResponseCallback = std::function<void(ErrorCode, const Response&)>;

/** @brief MCTP request structure
 *
 * Represents a request to be sent to an MCTP endpoint.
 * This structure contains all necessary information for sending and tracking a
 * request.
 */
struct Request
{
  public:
    /** @brief Constructor
     *
     * @param[in] data - The request data to be sent
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

    /** @brief Message type */
    uint8_t msgType{};

    /** @brief Response callback */
    ResponseCallback callback;

    /** @brief Timer for request timeout */
    std::unique_ptr<boost::asio::steady_timer> timer;

    /** @brief Request data payload
     *
     * The actual data to be sent in the MCTP request.
     */
    std::vector<uint8_t> vec;
};

/** @brief OCP AMI VDM request */
struct OcpVdmRequest : public Request
{
  public:
    /** @brief Constructor
     *
     * @param[in] data - The request data to be sent
     * @param[in] callback - The callback to invoke when the response is
     * received
     */
    explicit OcpVdmRequest(std::error_code& err, std::vector<uint8_t>&& data,
                           ResponseCallback&& callback) noexcept :
        Request(std::move(data), std::move(callback))
    {
        msgType = ocp::accelerator_management::messageType;

        if (vec.size() < sizeof(ocp::accelerator_management::BindingPciVid))
        {
            err = std::make_error_code(std::errc::bad_message);
            lg2::error(
                "OcpVdmRequest: Message too small - expected={ES}, received={RS}",
                "ES", sizeof(ocp::accelerator_management::BindingPciVid), "RS",
                vec.size());
            return;
        }

        // Get the instance ID from the request
        const auto* reqMsg =
            std::bit_cast<const ocp::accelerator_management::Message*>(
                vec.data());

        instanceId = reqMsg->hdr.instance_id &
                     ocp::accelerator_management::instanceIdBitMask;
    }

    /** @brief Instance ID */
    uint8_t instanceId;
};

/**
 * @brief Concept for MCTP socket interfaces
 *
 * An MCTP socket interface must provide methods for:
 * - Initializing the socket
 * - Sending a request
 * - Receiving a response
 */
template <typename T>
concept SocketInterface =
    requires(T socket, eid_t eid, const Request& request) {
        { socket.init() } -> std::same_as<std::weak_ptr<SocketFd>>;
        { socket.send(eid, request) } -> std::same_as<bool>;
        {
            socket.recv()
        } -> std::same_as<std::pair<eid_t, std::unique_ptr<Response>>>;
    };

/** @brief MCTP socket callback type
 *
 * Function type for handling received MCTP socket responses.
 * @param error - The error code from the socket operation
 * @param eid - The endpoint ID that sent the response
 * @param response - The response data received
 */
using MctpSocketCallback = std::function<void(
    const boost::system::error_code&, const eid_t&, std::unique_ptr<Response>)>;

/** @brief Concept for a Socket
 *
 * The socket must:
 * - Inherit from boost::asio::local::datagram_protocol::socket
 * - Provide methods for sending requests and receiving responses
 */
template <typename T>
concept Socket = requires(T socket, eid_t eid, const Request& request,
                          const MctpSocketCallback& callback) {
                     { socket.send(eid, request) } -> std::same_as<bool>;
                     { socket.recv(callback) };
                 };

} // namespace mctp
