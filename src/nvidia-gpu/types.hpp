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

/** @brief MCTP response callback type
 *
 * Function type for handling MCTP responses.
 * @param errorCode - The error code indicating success or failure
 * @param response - The response data received from the MCTP endpoint
 */
using ResponseCallback = std::function<void(int, const std::vector<uint8_t>&)>;

/** @brief MCTP request structure
 *
 * Represents a request to be sent to an MCTP endpoint.
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
    explicit Request(uint8_t msgtype, std::vector<uint8_t>&& data) noexcept :
        msgtype(msgtype), vec(std::move(data)) {};

    Request() = delete;
    Request(const Request& r) = delete;

    Request(Request&& r) noexcept = default;
    Request& operator=(Request&& r) noexcept = default;
    ~Request() noexcept = default;

    /** @brief Message type */
    uint8_t msgtype;

    /** @brief Request data payload
     *
     * The actual data to be sent in the MCTP request.
     */
    std::vector<uint8_t> vec;
};

/** @brief OCP AMI VDM request
 *
 * This structure contains all necessary information for sending and tracking a
 * request.
 */
struct OcpVdmRequest : public Request
{
  public:
    /** @brief Constructor
     *
     * @param[in] err - Error code if there are any failures while initializing
     * the Request
     * @param[in] data - The request data to be sent
     * @param[in] callback - The callback to invoke when the response is
     * received
     */
    explicit OcpVdmRequest(std::error_code& err, std::vector<uint8_t>&& data,
                           ResponseCallback&& callback) noexcept :
        Request(ocp::accelerator_management::messageType, std::move(data)),
        callback(std::move(callback))
    {
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
        const auto* reqmsg =
            std::bit_cast<const ocp::accelerator_management::Message*>(
                vec.data());

        instanceId = reqmsg->hdr.instance_id &
                     ocp::accelerator_management::instanceIdBitMask;
    }

    OcpVdmRequest() = delete;
    OcpVdmRequest(const OcpVdmRequest& r) = delete;

    OcpVdmRequest(OcpVdmRequest&& r) noexcept = default;
    OcpVdmRequest& operator=(OcpVdmRequest&& r) noexcept = default;
    ~OcpVdmRequest() noexcept = default;

    /** @brief Instance ID */
    uint8_t instanceId;

    /** @brief Response callback */
    ResponseCallback callback;

    /** @brief Timer for request timeout */
    std::unique_ptr<boost::asio::steady_timer> timer;
};

/** @brief MCTP socket callback type
 *
 * Function type for handling received MCTP socket responses.
 * @param error - The error code from the socket operation
 * @param eid - The endpoint ID that sent the response
 * @param response - The response data received
 */
using MctpSocketCallback = std::function<void(
    const std::error_code&, const eid_t&, const std::vector<uint8_t>&)>;

/** @brief Concept for a Socket
 *
 * The socket must provide methods for:
 * - Sending requests
 * - Receiving responses
 */
template <typename T>
concept Socket = requires(T socket, eid_t eid, const Request& request,
                          const MctpSocketCallback& callback) {
                     { socket.send(eid, request) } -> std::same_as<bool>;
                     { socket.recv(callback) };
                 };

} // namespace mctp
