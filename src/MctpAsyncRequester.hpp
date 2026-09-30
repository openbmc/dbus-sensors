/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/fdio.hpp>
#include <sdbusplus/async/mutex.hpp>
#include <sdbusplus/async/task.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <map>
#include <optional>
#include <span>
#include <system_error>
#include <vector>

namespace mctp
{

// Rq/D/instance-ID byte layout, as in the DSP0236 control message header.
constexpr uint8_t instanceIdBitMask = 0b00011111;
constexpr uint8_t datagramBitMask = 0b01000000;
constexpr uint8_t requestBitMask = 0b10000000;

// The parts of an OCP VDM binding the transport itself has to understand:
// which MCTP message type to bind to, and where the instance ID sits. The
// values belong to the protocol being spoken, so each one defines its own.
struct VdmBinding
{
    uint8_t msgType;
    size_t headerSize;
    size_t instanceIdOffset;
};

inline constexpr auto defaultResponseTimeout = std::chrono::seconds{2};

// Owns the AF_MCTP socket for one VDM message type and turns a request into an
// awaitable response.
//
// Requests are serialised over the socket, which is both what keeps fdio to the
// single awaiting task it documents and what lets a response be read straight
// out of the socket instead of being matched back to a request.
class AsyncRequester
{
  public:
    AsyncRequester() = delete;
    AsyncRequester(const AsyncRequester&) = delete;
    AsyncRequester(AsyncRequester&&) = delete;
    AsyncRequester& operator=(const AsyncRequester&) = delete;
    AsyncRequester& operator=(AsyncRequester&&) = delete;
    ~AsyncRequester() = default;

    explicit AsyncRequester(
        sdbusplus::async::context& ctx, VdmBinding binding,
        std::chrono::microseconds timeout = defaultResponseTimeout);

    using Response = std::expected<std::span<const uint8_t>, std::errc>;

    // The instance ID carried in reqMsg is replaced with one this owns. The
    // response is only valid until the next request.
    auto sendRecvMsg(uint8_t eid, std::span<const uint8_t> reqMsg)
        -> sdbusplus::async::task<Response>;

  private:
    struct Socket
    {
        explicit Socket(int fd) : fd(fd) {}
        Socket(const Socket&) = delete;
        Socket(Socket&&) = delete;
        Socket& operator=(const Socket&) = delete;
        Socket& operator=(Socket&&) = delete;
        ~Socket();

        int fd;
    };

    auto recvMsg(uint8_t eid, uint8_t iid) -> sdbusplus::async::task<Response>;
    std::optional<std::errc> sendMsg(uint8_t eid,
                                     std::span<const uint8_t> msg) const;
    uint8_t nextIid(uint8_t eid);

    VdmBinding binding;
    Socket socket;
    sdbusplus::async::fdio fdio;
    sdbusplus::async::mutex socketMutex{"mctp::AsyncRequester"};
    std::map<uint8_t, uint8_t> iids;
    std::vector<uint8_t> request;
    std::vector<uint8_t> response;
};

} // namespace mctp
