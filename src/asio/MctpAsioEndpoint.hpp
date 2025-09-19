#pragma once

#include <boost/asio/generic/datagram_protocol.hpp>

#include <optional>

// NOLINTNEXTLINE(readability-identifier-naming)
struct sockaddr_mctp;

// Wrapper around boost::asio::generic::datagram_protocol::endpoint to provide
// MCTP specific APIs that are available to the kernel
struct MctpAsioEndpoint
{
    MctpAsioEndpoint(uint8_t eid, uint8_t msgType);

    // usable for binding to any eid and a given msgType
    MctpAsioEndpoint(uint8_t msgType);

    MctpAsioEndpoint() = default;
    MctpAsioEndpoint(const MctpAsioEndpoint&) = delete;
    MctpAsioEndpoint(MctpAsioEndpoint&&) = delete;
    MctpAsioEndpoint& operator=(const MctpAsioEndpoint&) = delete;
    MctpAsioEndpoint& operator=(MctpAsioEndpoint&&) = delete;
    ~MctpAsioEndpoint() = default;
    boost::asio::generic::datagram_protocol::endpoint endpoint;

    std::optional<uint8_t> eid() const;

    std::optional<uint8_t> type() const;

  private:
    const struct sockaddr_mctp* getSockAddr() const;
};
