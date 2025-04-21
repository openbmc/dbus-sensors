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
#include <span>

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
} // namespace mctp
