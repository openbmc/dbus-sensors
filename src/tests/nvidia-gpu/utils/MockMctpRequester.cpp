/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Compiled instead of the real MctpRequester.cpp: same ABI, but no AF_MCTP
 * socket, and sendRecvMsg() forwards to the registered MctpRequesterMock.
 */

#include "MockMctpRequester.hpp"

#include "MctpRequester.hpp"

#include <sys/socket.h>

#include <boost/asio/generic/datagram_protocol.hpp>
#include <boost/asio/io_context.hpp>

#include <cstdint>
#include <functional>
#include <span>
#include <system_error>
#include <utility>

#include <gtest/gtest.h>

namespace
{
mock_mctp::MctpRequesterMock* activeMock = nullptr;
} // namespace

namespace mock_mctp
{

void setActiveMock(MctpRequesterMock* mock)
{
    activeMock = mock;
}

} // namespace mock_mctp

namespace mctp
{

MctpRequester::MctpRequester(boost::asio::io_context& ctx) :
    io{ctx},
    // Open a dummy UDP socket so the member is valid but never used.
    mctpSocket(ctx, boost::asio::generic::datagram_protocol{AF_INET, 0})
{}

// Signature fixed by the real MctpRequester.hpp.
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
void MctpRequester::sendRecvMsg(
    uint8_t eid, std::span<const uint8_t> reqMsg,
    std::move_only_function<void(const std::error_code&,
                                 std::span<const uint8_t>)>
        callback)
{
    if (activeMock == nullptr)
    {
        ADD_FAILURE() << "MctpRequester::sendRecvMsg called with no active "
                         "mock; register one with mock_mctp::setActiveMock()";
        return;
    }
    activeMock->sendRecvMsg(eid, reqMsg, std::move(callback));
}

} // namespace mctp
