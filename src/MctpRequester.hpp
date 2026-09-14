/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <MctpAsioEndpoint.hpp>
#include <boost/asio/generic/datagram_protocol.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/container/devector.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/small_vector.hpp>

#include <cstddef>
#include <cstdint>
#include <expected>
#include <functional>
#include <iostream>
#include <memory>
#include <queue>
#include <span>
#include <system_error>
#include <unordered_map>
#include <utility>

namespace mctp
{

// Rq/D/instance-ID byte layout, as in the DSP0236 control message header.
constexpr uint8_t instanceIdBitMask = 0b00011111;
constexpr uint8_t datagramBitMask = 0b01000000;
constexpr uint8_t requestBitMask = 0b10000000;

// The parts of an OCP VDM binding the transport itself has to understand:
// which MCTP message type to bind to, and where the instance ID sits.
struct VdmBinding
{
    uint8_t msgType;
    size_t headerSize;
    size_t instanceIdOffset;
};

// DSP0236 vendor-defined, PCI binding.
inline constexpr VdmBinding ocpVdmPciBinding{
    .msgType = 0x7E,
    .headerSize = 5,
    .instanceIdOffset = 2,
};

using MctpEventHandler =
    std::move_only_function<void(uint8_t eid, std::span<const uint8_t> msg)>;

class MctpRequester
{
  public:
    MctpRequester() = delete;

    MctpRequester(const MctpRequester&) = delete;

    MctpRequester(MctpRequester&&) = delete;

    MctpRequester& operator=(const MctpRequester&) = delete;

    MctpRequester& operator=(MctpRequester&&) = delete;

    MctpRequester(boost::asio::io_context& ctx, VdmBinding binding,
                  MctpEventHandler eventHandler);

    void sendRecvMsg(uint8_t eid, std::span<const uint8_t> reqMsg,
                     std::move_only_function<void(const std::error_code&,
                                                  std::span<const uint8_t>)>
                         callback);

  private:
    using cb_t = std::move_only_function<void(const std::error_code&,
                                              std::span<const uint8_t>)>;

    struct RequestContext
    {
        std::vector<uint8_t> reqMsg;
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

    VdmBinding binding;
    MctpEventHandler eventHandler;

    boost::asio::generic::datagram_protocol::socket mctpSocket;
    static constexpr size_t maxMessageSize = 65536 + 256;
    std::array<uint8_t, maxMessageSize> buffer{};
    MctpAsioEndpoint recvEndPoint;
    std::unordered_map<uint8_t, EidContext> requestContextQueues;
};

} // namespace mctp
