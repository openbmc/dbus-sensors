/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpAsyncRequester.hpp"

#include <sdbusplus/async.hpp>

// Because of issues with glibc not matching linux, these have to come after
// anything that implicitly pulls in the system network headers.
// clang-format off
#include <linux/mctp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
// clang-format on

#include <bit>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <system_error>

namespace mctp
{

namespace
{

// The largest MCTP message, with room for the binding header.
constexpr size_t maxMessageSize = 65536 + 256;

sockaddr_mctp addressOf(uint8_t eid, uint8_t msgType)
{
    sockaddr_mctp addr{};
    addr.smctp_family = AF_MCTP;
    addr.smctp_network = MCTP_NET_ANY;
    addr.smctp_addr.s_addr = eid;
    addr.smctp_type = msgType;
    addr.smctp_tag = MCTP_TAG_OWNER;
    return addr;
}

int openSocket()
{
    const int fd = ::socket(AF_MCTP, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd < 0)
    {
        throw std::system_error(errno, std::generic_category(),
                                "AF_MCTP socket");
    }

    return fd;
}

} // namespace

AsyncRequester::Socket::~Socket()
{
    ::close(fd);
}

AsyncRequester::AsyncRequester(sdbusplus::async::context& ctx,
                               VdmBinding binding,
                               std::chrono::microseconds timeout) :
    binding(binding), socket(openSocket()), fdio(ctx, socket.fd, timeout),
    response(maxMessageSize)
{}

uint8_t AsyncRequester::nextIid(uint8_t eid)
{
    uint8_t& iid = iids[eid];
    iid = static_cast<uint8_t>((iid + 1) & instanceIdBitMask);
    return iid;
}

std::optional<std::errc> AsyncRequester::sendMsg(
    uint8_t eid, std::span<const uint8_t> msg) const
{
    sockaddr_mctp addr = addressOf(eid, binding.msgType);
    const ssize_t sent =
        ::sendto(socket.fd, msg.data(), msg.size(), 0,
                 std::bit_cast<sockaddr*>(&addr), sizeof(addr));
    if (sent < 0)
    {
        return static_cast<std::errc>(errno);
    }

    if (static_cast<size_t>(sent) != msg.size())
    {
        return std::errc::message_size;
    }

    return std::nullopt;
}

auto AsyncRequester::sendRecvMsg(uint8_t eid, std::span<const uint8_t> reqMsg)
    -> sdbusplus::async::task<Response>
{
    if (reqMsg.size() < binding.headerSize)
    {
        co_return std::unexpected(std::errc::invalid_argument);
    }

    sdbusplus::async::lock_guard guard{socketMutex};
    co_await guard.lock();

    request.assign(reqMsg.begin(), reqMsg.end());
    uint8_t& control = request[binding.instanceIdOffset];
    const uint8_t iid = nextIid(eid);
    control = static_cast<uint8_t>((control & ~instanceIdBitMask) | iid);

    if (auto error = sendMsg(eid, request))
    {
        co_return std::unexpected(*error);
    }

    co_return co_await recvMsg(eid, iid);
}

auto AsyncRequester::recvMsg(uint8_t eid, uint8_t iid)
    -> sdbusplus::async::task<Response>
{
    while (true)
    {
        try
        {
            co_await fdio.next();
        }
        catch (const sdbusplus::async::fdio_timeout_exception&)
        {
            co_return std::unexpected(std::errc::timed_out);
        }

        sockaddr_mctp from{};
        socklen_t fromLen = sizeof(from);
        const ssize_t length =
            ::recvfrom(socket.fd, response.data(), response.size(), 0,
                       std::bit_cast<sockaddr*>(&from), &fromLen);
        if (length < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                continue;
            }
            co_return std::unexpected(static_cast<std::errc>(errno));
        }

        const std::span<const uint8_t> msg(response.data(),
                                           static_cast<size_t>(length));
        if (msg.size() < binding.headerSize ||
            from.smctp_type != binding.msgType || from.smctp_addr.s_addr != eid)
        {
            continue;
        }

        const uint8_t control = msg[binding.instanceIdOffset];
        if ((control & (requestBitMask | datagramBitMask)) != 0 ||
            (control & instanceIdBitMask) != iid)
        {
            continue;
        }

        co_return msg;
    }
}

} // namespace mctp
