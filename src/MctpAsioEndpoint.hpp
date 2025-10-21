#pragma once

#include <boost/asio/generic/datagram_protocol.hpp>

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

#include <optional>

// Wrapper around boost::asio::generic::datagram_protocol::endpoint to provide
// MCTP specific APIs that are available to the kernel
struct MctpAsioEndpoint
{
    MctpAsioEndpoint();

    MctpAsioEndpoint(uint8_t eid, uint8_t msgType);

    MctpAsioEndpoint(const MctpAsioEndpoint&) = delete;
    MctpAsioEndpoint(MctpAsioEndpoint&&) = delete;
    MctpAsioEndpoint& operator=(const MctpAsioEndpoint&) = delete;
    MctpAsioEndpoint& operator=(MctpAsioEndpoint&&) = delete;

    boost::asio::generic::datagram_protocol::endpoint endpoint;

    std::optional<uint8_t> eid() const;

    std::optional<uint8_t> type() const;

  private:
    const struct sockaddr_mctp* getSockAddr() const;
};
