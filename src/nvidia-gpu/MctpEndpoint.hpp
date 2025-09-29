#include <boost/asio/generic/datagram_protocol.hpp>
#include <phosphor-logging/lg2.hpp>

#include <utility>

// Becuase of issues with glibc not matching linux, we need to make sure these
// are included AFTER the system headers, which are implictly included by boost.
// https://bugzilla.redhat.com/show_bug.cgi?id=1300256
// https://github.com/systemd/systemd/commit/08ce521fb2546921f2642bef067d2cc02158b121
// https://github.com/systemd/systemd/issues/2864
// clang-format off
#include <linux/mctp.h>
#include <sys/socket.h>
// clang-format on

// Wrapper around boost::asio::generic::datagram_protocol::endpoint to provide
// MCTP specific APIs
struct MctpEndpoint
{
    MctpEndpoint() = default;
    MctpEndpoint(const MctpEndpoint&) = delete;
    MctpEndpoint(MctpEndpoint&&) = delete;
    MctpEndpoint& operator=(const MctpEndpoint&) = delete;
    MctpEndpoint& operator=(MctpEndpoint&&) = delete;

    boost::asio::generic::datagram_protocol::endpoint endpoint;

    MctpEndpoint(boost::asio::generic::datagram_protocol::endpoint endpoint) :
        endpoint(std::move(endpoint))
    {}

    uint8_t eid() const
    {
        if (endpoint.size() < sizeof(struct sockaddr_mctp))
        {
            lg2::error("MctpRequester: Received endpoint is too small?");
            return std::numeric_limits<uint8_t>::max();
        }

        struct sockaddr_mctp* respAddr = getSockAddr();
        if (respAddr == nullptr)
        {
            return std::numeric_limits<uint8_t>::max();
        }

        return respAddr->smctp_addr.s_addr;
    }

    uint8_t type() const
    {
        struct sockaddr_mctp* respAddr = getSockAddr();
        if (respAddr == nullptr)
        {
            return std::numeric_limits<uint8_t>::max();
        }
        return respAddr->smctp_type;
    }

  private:
    struct sockaddr_mctp* getSockAddr() const
    {
        if (endpoint.size() < sizeof(struct sockaddr_mctp))
        {
            lg2::error("MctpRequester: Received endpoint is too small?");
            return nullptr;
        }

        return std::bit_cast<struct sockaddr_mctp*>(endpoint.data());
    }
};
