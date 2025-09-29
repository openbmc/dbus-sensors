/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <MctpAsioEndpoint.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/generic/datagram_protocol.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/devector.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/small_vector.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <unordered_map>
#include <utility>

namespace mctp
{

class MctpRequester
{
  public:
    MctpRequester() = delete;
    MctpRequester(const MctpRequester&) = delete;
    MctpRequester(MctpRequester&&) = delete;
    MctpRequester& operator=(const MctpRequester&) = delete;
    MctpRequester& operator=(MctpRequester&&) = delete;

    explicit MctpRequester(boost::asio::io_context& ctx);

    void sendRecvMsg(
        uint8_t eid, std::span<const uint8_t> reqMsg,
        std::move_only_function<void(int, std::span<const uint8_t>)>&&
            callback);

  private:
    struct RequestContext
    {
        boost::container::small_vector<uint8_t, 32> reqMsg;
        std::move_only_function<void(int, std::span<const uint8_t>)> callback;

        RequestContext(const RequestContext&) = delete;
        RequestContext& operator=(const RequestContext&) = delete;

        RequestContext(RequestContext&&) = default;
        RequestContext& operator=(RequestContext&&) = default;
        ~RequestContext() = default;

        explicit RequestContext(
            std::span<const uint8_t> req,
            std::move_only_function<void(int, std::span<const uint8_t>)>&& cb) :
            reqMsg(req.begin(), req.end()), callback(std::move(cb))
        {}
    };

    void handleResult(uint8_t eid, int result,
                      std::span<const uint8_t> response);
    void processQueue(uint8_t eid);

    void startReceiving();

    static uint8_t getNextInstanceId();

    void processRecvMsg(const boost::system::error_code& ec, size_t length);

    void handleSendMsgCompletion(uint8_t eid,
                                 const boost::system::error_code& ec,
                                 size_t length);

    MctpAsioEndpoint recvEndPoint;

    struct CompletionMatch
    {
        uint8_t eid;
        uint8_t instanceId;
        auto operator<=>(const CompletionMatch&) const = default;
        bool operator==(const CompletionMatch&) const = default;
    };

    // Queue of requests for each EID
    std::unordered_map<uint8_t, boost::container::devector<RequestContext>>
        requestContextQueues;

    // Map of completion callbacks for each request
    boost::container::flat_map<
        CompletionMatch,
        std::move_only_function<void(int, std::span<const uint8_t>)>>
        completionCallbacks;

    std::array<uint8_t, 256> responseBuffer{};

    boost::asio::generic::datagram_protocol::socket mctpSocket;

    boost::asio::steady_timer expiryTimer;
};

} // namespace mctp
