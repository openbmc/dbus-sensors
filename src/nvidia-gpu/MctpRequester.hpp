/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <OcpMctpVdm.hpp>
#include <boost/asio/generic/datagram_protocol.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <queue>
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

    explicit Requester(boost::asio::io_context& ctx);

    void sendRecvMsg(uint8_t eid, std::span<const uint8_t> reqMsg,
                     std::span<uint8_t> respMsg,
                     std::move_only_function<void(int)> callback);

  private:
    void processRecvMsg(uint8_t eid, std::span<const uint8_t> reqMsg,
                        std::span<uint8_t> respMsg,
                        const boost::system::error_code& ec, size_t length);

    void handleSendMsgCompletion(uint8_t eid, std::span<const uint8_t> reqMsg,
                                 std::span<uint8_t> respMsg,
                                 const boost::system::error_code& ec,
                                 size_t length);

    boost::asio::generic::datagram_protocol::socket mctpSocket;

    static constexpr size_t maxMessageSize = 65536 + 256;

    boost::asio::generic::datagram_protocol::endpoint sendEndPoint;

    boost::asio::generic::datagram_protocol::endpoint recvEndPoint;

    boost::asio::steady_timer expiryTimer;

    std::move_only_function<void(int)> completionCallback;

    static constexpr uint8_t msgType = ocp::accelerator_management::messageType;
};

class QueuingRequester
{
  public:
    QueuingRequester() = delete;
    QueuingRequester(const QueuingRequester&) = delete;
    QueuingRequester(QueuingRequester&&) = delete;
    QueuingRequester& operator=(const QueuingRequester&) = delete;
    QueuingRequester& operator=(QueuingRequester&&) = delete;

    explicit QueuingRequester(boost::asio::io_context& ctx) : requester(ctx) {}

    void sendRecvMsg(uint8_t eid, std::span<const uint8_t> reqMsg,
                     std::span<uint8_t> respMsg,
                     std::move_only_function<void(int)> callback);

  private:
    struct RequestContext
    {
        std::span<const uint8_t> reqMsg;
        std::span<uint8_t> respMsg;
        std::move_only_function<void(int)> callback;

        RequestContext(const RequestContext&) = delete;
        RequestContext& operator=(const RequestContext&) = delete;

        RequestContext(RequestContext&&) = default;
        RequestContext& operator=(RequestContext&&) = default;
        ~RequestContext() = default;

        explicit RequestContext(std::span<const uint8_t> req,
                                std::span<uint8_t> resp,
                                std::move_only_function<void(int)> cb) :
            reqMsg(req), respMsg(resp), callback(std::move(cb))
        {}
    };

    void processNextRequest(uint8_t eid);

    Requester requester;
    std::unordered_map<uint8_t, std::queue<std::unique_ptr<RequestContext>>>
        requestContextQueue;
};

using MctpRequester = QueuingRequester;
} // namespace mctp
