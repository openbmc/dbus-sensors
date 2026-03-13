/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Mock implementation of mctp::MctpRequester for unit tests.
 *
 * This file is compiled INSTEAD of the real MctpRequester.cpp.
 * It provides the same ABI but:
 *   - The constructor does NOT open a real AF_MCTP socket.
 *   - sendRecvMsg() invokes the callback synchronously with a
 *     configurable response set via mock_mctp helpers.
 *
 * Supports two modes:
 *   1. Single response: setNextResponse() — same response for every call.
 *   2. Queued responses: pushResponse() — each call pops the next response.
 *      When the queue is empty, falls back to the single response.
 */

#include "MctpRequester.hpp"

#include <sys/socket.h>

#include <boost/asio/io_context.hpp>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <span>
#include <system_error>
#include <utility>
#include <vector>

// ─── Global mock state ──────────────────────────────────────────────
namespace mock_mctp
{

struct Response
{
    std::error_code ec;
    std::vector<uint8_t> data;
};

static Response defaultResponse;
static std::deque<Response> responseQueue;
static std::vector<std::vector<uint8_t>> requestHistory;
static size_t requestCount = 0;
static uint8_t lastEid = 0;

void setNextResponse(std::error_code ec, std::vector<uint8_t> response)
{
    defaultResponse.ec = ec;
    defaultResponse.data = std::move(response);
}

void pushResponse(std::error_code ec, std::vector<uint8_t> response)
{
    responseQueue.push_back({ec, std::move(response)});
}

void clearNextResponse()
{
    defaultResponse = {};
    responseQueue.clear();
}

void clearHistory()
{
    requestHistory.clear();
    requestCount = 0;
    lastEid = 0;
}

const std::vector<std::vector<uint8_t>>& getRequestHistory()
{
    return requestHistory;
}

size_t getRequestCount()
{
    return requestCount;
}

uint8_t getLastEid()
{
    return lastEid;
}

std::span<const uint8_t> getLastRequest()
{
    if (requestHistory.empty())
    {
        return {};
    }
    return requestHistory.back();
}

} // namespace mock_mctp

// ─── MctpRequester stub implementation ──────────────────────────────
namespace mctp
{

// NOLINTNEXTLINE(cppcoreguidelines-pro-type-member-init)
MctpRequester::MctpRequester(boost::asio::io_context& ctx) :
    io{ctx},
    // Open a dummy UDP socket so the member is valid but never used.
    mctpSocket(ctx, boost::asio::generic::datagram_protocol{AF_INET, 0})
{}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
void MctpRequester::sendRecvMsg(
    uint8_t eid, std::span<const uint8_t> reqMsg,
    std::move_only_function<void(const std::error_code&,
                                 std::span<const uint8_t>)>
        callback)
{
    mock_mctp::requestHistory.emplace_back(reqMsg.begin(), reqMsg.end());
    mock_mctp::requestCount++;
    mock_mctp::lastEid = eid;

    if (!mock_mctp::responseQueue.empty())
    {
        auto resp = std::move(mock_mctp::responseQueue.front());
        mock_mctp::responseQueue.pop_front();
        callback(resp.ec, resp.data);
    }
    else
    {
        callback(mock_mctp::defaultResponse.ec,
                 mock_mctp::defaultResponse.data);
    }
}

} // namespace mctp
