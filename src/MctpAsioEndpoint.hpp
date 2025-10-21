#pragma once

#include <boost/asio/generic/datagram_protocol.hpp>

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
