#pragma once

#include <boost/asio/generic/datagram_protocol.hpp>
#include <phosphor-logging/lg2.hpp>

#include <optional>
#include <utility>

// Becuase of issues with glibc not matching linux, we need to make sure these
// are included AFTER the system headers, which are implictly included by boost.
// These show up as errors like
// /usr/include/net/if.h:44:14: error: ‘IFF_UP’ conflicts with a previous
// declaration
// The bugs below are other projects working around similar issues
// https://bugzilla.redhat.com/show_bug.cgi?id=1300256
// https://github.com/systemd/systemd/commit/08ce521fb2546921f2642bef067d2cc02158b121
// https://github.com/systemd/systemd/issues/2864
// clang-format off
#include <linux/mctp.h>
#include <sys/socket.h>
// clang-format on

// Wrapper around boost::asio::generic::datagram_protocol::endpoint to provide
// MCTP specific APIs that are available to the kernel
struct MctpAsioEndpoint
{
    MctpAsioEndpoint() = default;
    MctpAsioEndpoint(const MctpAsioEndpoint&) = delete;
    MctpAsioEndpoint(MctpAsioEndpoint&&) = delete;
    MctpAsioEndpoint& operator=(const MctpAsioEndpoint&) = delete;
    MctpAsioEndpoint& operator=(MctpAsioEndpoint&&) = delete;

    boost::asio::generic::datagram_protocol::endpoint endpoint;

    std::optional<uint8_t> eid() const
    {
        const struct sockaddr_mctp* sock = getSockAddr();
        if (sock == nullptr)
        {
            return std::nullopt;
        }
        return sock->smctp_addr.s_addr;
    }

    std::optional<uint8_t> type() const
    {
        const struct sockaddr_mctp* sock = getSockAddr();
        if (sock == nullptr)
        {
            return std::nullopt;
        }
        return sock->smctp_type;
    }

  private:
    const struct sockaddr_mctp* getSockAddr() const
    {
        if (endpoint.size() < sizeof(struct sockaddr_mctp))
        {
            lg2::error("MctpRequester: Received endpoint is too small?");
            return nullptr;
        }

        return std::bit_cast<struct sockaddr_mctp*>(endpoint.data());
    }
};
