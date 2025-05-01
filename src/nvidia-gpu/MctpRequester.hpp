/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "InstanceDb.hpp"

#include <OcpMctpVdm.hpp>
#include <boost/asio/generic/datagram_protocol.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/devector.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <unordered_map>
#include <utility>

namespace mctp
{
class Requester
{
  public:
    Requester() = delete;

    Requester(const Requester&) = delete;

    Requester(Requester&&) = delete;

    Requester& operator=(const Requester&) = delete;

    Requester& operator=(Requester&&) = delete;

    using ResponseHandler = std::move_only_function<void(
        uint8_t eid, std::span<uint8_t> respMsg, int errorCode)>;

    explicit Requester(boost::asio::io_context& ctx, uint8_t msgType,
                       ResponseHandler&& responseHandler);

    void sendRecvMsg(uint8_t eid, std::span<const uint8_t> reqMsg,
                     std::span<uint8_t> respMsg);

  private:
    void processRecvMsg(std::span<const uint8_t> reqMsg,
                        std::span<uint8_t> respMsg,
                        const boost::system::error_code& ec, size_t length);

    void handleSendMsgCompletion(uint8_t eid, std::span<const uint8_t> reqMsg,
                                 std::span<uint8_t> respMsg,
                                 const boost::system::error_code& ec,
                                 size_t length);

    boost::asio::generic::datagram_protocol::socket mctpSocket;

    boost::asio::generic::datagram_protocol::endpoint sendEndPoint;

    boost::asio::generic::datagram_protocol::endpoint recvEndPoint;

    ResponseHandler responseHandler;

    uint8_t msgType;
};

class NvidiaMctpVdmRequester
{
  public:
    NvidiaMctpVdmRequester() = delete;
    NvidiaMctpVdmRequester(const NvidiaMctpVdmRequester&) = delete;
    NvidiaMctpVdmRequester(NvidiaMctpVdmRequester&&) = delete;
    NvidiaMctpVdmRequester& operator=(const NvidiaMctpVdmRequester&) = delete;
    NvidiaMctpVdmRequester& operator=(NvidiaMctpVdmRequester&&) = delete;

    explicit NvidiaMctpVdmRequester(boost::asio::io_context& ctx);

    void sendRecvMsg(uint8_t eid, std::span<uint8_t> reqMsg,
                     std::span<uint8_t> respMsg,
                     std::move_only_function<void(int)> callback);

  private:
    struct RequestContext
    {
        std::span<uint8_t> reqMsg;
        std::span<uint8_t> respMsg;
        std::move_only_function<void(int)> callback;

        RequestContext(const RequestContext&) = delete;
        RequestContext& operator=(const RequestContext&) = delete;

        RequestContext(RequestContext&&) = default;
        RequestContext& operator=(RequestContext&&) = default;
        ~RequestContext() = default;

        explicit RequestContext(std::span<uint8_t> req, std::span<uint8_t> resp,
                                std::move_only_function<void(int)>&& cb) :
            reqMsg(req), respMsg(resp), callback(std::move(cb))
        {}

        // NOLINTNEXTLINE
        void setRequestInstanceId(uint8_t id)
        {
            auto* reqHdr =
                // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
                reinterpret_cast<ocp::accelerator_management::BindingPciVid*>(
                    reqMsg.data());

            // Clear old instance ID bits
            reqHdr->instance_id &=
                ~ocp::accelerator_management::instanceIdBitMask;

            // Set new instance ID
            reqHdr->instance_id |=
                (id & ocp::accelerator_management::instanceIdBitMask);
        }

        uint8_t getInstanceId() const
        {
            const auto* reqHdr =
                // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
                reinterpret_cast<
                    const ocp::accelerator_management::BindingPciVid*>(
                    reqMsg.data());
            return reqHdr->instance_id;
        }
    };

    void handleResult(uint8_t eid, int result);
    void processQueue(uint8_t eid);
    void handleResponse(uint8_t eid, std::span<uint8_t> respMsg, int ec);

    boost::asio::io_context& ctx;
    Requester requester;
    std::unordered_map<uint8_t, std::unique_ptr<boost::asio::steady_timer>>
        expiryTimers;
    std::unordered_map<
        uint8_t, boost::container::devector<std::unique_ptr<RequestContext>>>
        requestContextQueues;
    ocp::InstanceIdDb instanceIdDb;
};

using MctpRequester = NvidiaMctpVdmRequester;
} // namespace mctp
