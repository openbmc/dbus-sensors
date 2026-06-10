/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <MctpAsioEndpoint.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/generic/datagram_protocol.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/container/devector.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/small_vector.hpp>
#include <sdbusplus/message.hpp>

#include <cstddef>
#include <cstdint>
#include <expected>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <queue>
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

    // Issue an MCTP request and deliver the response (or error) to callback.
    //
    // dbusMethodReplier optionally carries the sdbusplus::message_t for a
    // D-Bus method call that is registered with deferred reply
    // (register_method(..., /*deferred=*/true)).  When present, handleResult
    // sends the reply for that call once the response arrives or the request
    // terminates: an empty method return on success, or a method-errno reply
    // on a transport error.  Leave it as std::nullopt for requests that are
    // not driven by a deferred D-Bus method.
    void sendRecvMsg(
        uint8_t eid, std::span<const uint8_t> reqMsg,
        std::move_only_function<void(const std::error_code&,
                                     std::span<const uint8_t>)>
            callback,
        std::optional<sdbusplus::message_t> dbusMethodReplier = std::nullopt);

  private:
    using cb_t = std::move_only_function<void(const std::error_code&,
                                              std::span<const uint8_t>)>;

    static constexpr uint8_t msgType = ocp::accelerator_management::messageType;

    struct RequestContext
    {
        std::vector<uint8_t> reqMsg;
        cb_t callback;

        // The call message for a deferred-reply D-Bus method, if this request
        // was initiated from one; std::nullopt otherwise.  handleResult sends
        // the method reply through it when it has a value.
        std::optional<sdbusplus::message_t> dbusMethodReplier;

        RequestContext(const RequestContext&) = delete;
        RequestContext& operator=(const RequestContext&) = delete;

        RequestContext(RequestContext&&) = default;
        RequestContext& operator=(RequestContext&&) = default;
        ~RequestContext() = default;

        RequestContext(std::span<const uint8_t> req, cb_t&& cb,
                       std::optional<sdbusplus::message_t>&& replier) :
            reqMsg(req.begin(), req.end()), callback(std::move(cb)),
            dbusMethodReplier(std::move(replier))
        {}
    };

    struct EidContext
    {
        boost::asio::steady_timer timer;
        uint8_t iid{};
        boost::container::devector<RequestContext> queue;
        EidContext(boost::asio::io_context& io) : timer{io}, iid{0xFF} {}
        EidContext(EidContext&&) noexcept = default;
        EidContext& operator=(EidContext&&) noexcept = default;
        EidContext& operator=(const EidContext&) = delete;
        EidContext(const EidContext&) = delete;
        ~EidContext() = default;
    };

    std::optional<uint8_t> getNextIid(uint8_t eid);
    void startReceive();
    void processRecvMsg(const boost::system::error_code& ec, size_t length);
    void handleSendMsgCompletion(uint8_t eid,
                                 const boost::system::error_code& ec,
                                 size_t length);

    void handleResult(uint8_t eid, const std::error_code& ec,
                      std::span<const uint8_t> buffer);
    void processQueue(uint8_t eid);

    boost::asio::io_context& io;

    boost::asio::generic::datagram_protocol::socket mctpSocket;
    static constexpr size_t maxMessageSize = 65536 + 256;
    std::array<uint8_t, maxMessageSize> buffer{};
    MctpAsioEndpoint recvEndPoint;
    std::unordered_map<uint8_t, EidContext> requestContextQueues;
};

} // namespace mctp
