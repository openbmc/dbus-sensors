/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "OcpMctpVdm.hpp"

#include <boost/asio/steady_timer.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace mctp
{
/** @brief MCTP endpoint ID type */
using eid_t = uint8_t;

/** @brief MCTP response */
using Response = std::vector<uint8_t>;

/** @brief ErrorCode */
using ErrorCode = int;

/** @brief Socket file descriptor */
using SocketFd = int;

/** @brief MCTP response callback */
using ResponseCallback = std::function<void(ErrorCode, const Response&)>;

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

    /** @brief Message type */
    uint8_t msgType{};

    /** @brief Response callback */
    ResponseCallback callback;

    /** @brief Timer for request timeout */
    std::unique_ptr<boost::asio::steady_timer> timer;

    /** @brief Request data */
    std::vector<uint8_t> vec;
};

/** @brief OCP AMI VDM request */
struct OcpVdmRequest : public Request
{
  public:
    /** @brief Constructor
     *
     * @param[in] data - The request data
     * @param[in] callback - The callback to invoke when the response is
     * received
     */
    explicit OcpVdmRequest(std::vector<uint8_t>&& data,
                           ResponseCallback&& callback) noexcept :
        Request(std::move(data), std::move(callback))
    {
        msgType = ocp::accelerator_management::messageType;

        if (vec.size() >= sizeof(ocp::accelerator_management::BindingPciVid))
        {
            // Get the instance ID from the request
            const auto* reqMsg =
                std::bit_cast<const ocp::accelerator_management::Message*>(
                    vec.data());

            instanceId = reqMsg->hdr.instance_id;
        }
    };

    /** @brief Instance ID */
    uint8_t instanceId{};
};
} // namespace mctp
