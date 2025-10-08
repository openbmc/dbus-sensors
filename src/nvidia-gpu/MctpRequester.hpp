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
#include <boost/container/devector.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/small_vector.hpp>

#include <cstddef>
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <span>
#include <system_error>
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

    void sendRecvMsg(uint8_t eid, std::span<const uint8_t> reqMsg,
                     std::move_only_function<void(const std::error_code&,
                                                  std::span<const uint8_t>)>
                         callback);

  private:
    using cb_t = std::move_only_function<void(const std::error_code&,
                                              std::span<const uint8_t>)>;

    void processRecvMsg(const boost::system::error_code& ec, size_t length);

    void handleSendMsgCompletion(uint8_t eid,
                                 const boost::system::error_code& ec,
                                 size_t length);

    struct RequestContext
    {
        boost::container::small_vector<uint8_t, 32> reqMsg;
        cb_t callback;

        RequestContext(const RequestContext&) = delete;
        RequestContext& operator=(const RequestContext&) = delete;

        RequestContext(RequestContext&&) = default;
        RequestContext& operator=(RequestContext&&) = default;
        ~RequestContext() = default;

        explicit RequestContext(std::span<const uint8_t> req, cb_t&& cb) :
            reqMsg(req.begin(), req.end()), callback(std::move(cb))
        {}
    };

    struct EidContext
    {
        uint8_t iid{};
        boost::container::devector<std::unique_ptr<RequestContext>> queue;
    };

    std::expected<uint8_t, std::error_code> getNextIid(uint8_t eid);
    void bindReceive();
    boost::asio::generic::datagram_protocol::socket mctpSocket;

    static constexpr size_t maxMessageSize = 65536 + 256;
    std::array<uint8_t, maxMessageSize> buffer{};

    boost::asio::generic::datagram_protocol::endpoint sendEndPoint;

    boost::asio::generic::datagram_protocol::endpoint recvEndPoint;

    boost::asio::steady_timer expiryTimer;

    void handleResult(uint8_t eid, const std::error_code& ec,
                      std::span<const uint8_t> buffer);

    void processQueue(uint8_t eid);

    // Queue of requests for each EID
    std::unordered_map<uint8_t, EidContext> requestContextQueues;

    static constexpr uint8_t msgType = ocp::accelerator_management::messageType;
};

} // namespace mctp
