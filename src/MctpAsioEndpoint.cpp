#include "MctpAsioEndpoint.hpp"

#include <boost/asio/generic/datagram_protocol.hpp>
#include <phosphor-logging/lg2.hpp>

#include <bit>
#include <cstdint>
#include <optional>

// Because of issues with glibc not matching linux, we need to make sure these
// are included AFTER the system headers, which are implicitly included by boost.
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

MctpAsioEndpoint::MctpAsioEndpoint(uint8_t eid, uint8_t msgType)
{
    endpoint.resize(sizeof(struct sockaddr_mctp));
    struct sockaddr_mctp* sock =
        std::bit_cast<struct sockaddr_mctp*>(endpoint.data());
    sock->smctp_addr.s_addr = eid;
    sock->smctp_family = AF_MCTP;
    sock->smctp_type = msgType;
    sock->smctp_tag = MCTP_TAG_OWNER;
    sock->smctp_network = MCTP_NET_ANY;
};

MctpAsioEndpoint::MctpAsioEndpoint(uint8_t msgType) :
    MctpAsioEndpoint{MCTP_ADDR_ANY, msgType}
{}

std::optional<uint8_t> MctpAsioEndpoint::eid() const
{
    const struct sockaddr_mctp* sock = getSockAddr();
    if (sock == nullptr)
    {
        return std::nullopt;
    }
    return sock->smctp_addr.s_addr;
}

std::optional<uint8_t> MctpAsioEndpoint::type() const
{
    const struct sockaddr_mctp* sock = getSockAddr();
    if (sock == nullptr)
    {
        return std::nullopt;
    }
    return sock->smctp_type;
}

const struct sockaddr_mctp* MctpAsioEndpoint::getSockAddr() const
{
    if (endpoint.size() < sizeof(struct sockaddr_mctp))
    {
        lg2::error("MctpRequester: Received endpoint is too small?");
        return nullptr;
    }

    return std::bit_cast<struct sockaddr_mctp*>(endpoint.data());
}
