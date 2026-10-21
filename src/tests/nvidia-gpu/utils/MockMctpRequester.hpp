/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>
#include <deque>
#include <functional>
#include <span>
#include <system_error>
#include <utility>
#include <vector>

#include <gmock/gmock.h>

/**
 * gmock seam for the link-time mock of mctp::MctpRequester: the stub in
 * MockMctpRequester.cpp forwards every sendRecvMsg() call to the mock
 * registered via setActiveMock().
 */
namespace mock_mctp
{

class MctpRequesterMock
{
  public:
    MOCK_METHOD(void, sendRecvMsg,
                (uint8_t eid, std::span<const uint8_t> reqMsg,
                 std::move_only_function<void(const std::error_code&,
                                              std::span<const uint8_t>)>
                     callback));
};

/** Register the forwarding target (nullptr to unregister). */
void setActiveMock(MctpRequesterMock* mock);

/** Action that completes the request with (ec, response); owns the bytes
 *  so the span handed to the callback cannot dangle. */
inline auto respondWith(std::error_code ec, std::vector<uint8_t> response)
{
    return [ec, response = std::move(response)](
               uint8_t /*eid*/, std::span<const uint8_t> /*reqMsg*/,
               auto callback) { callback(ec, response); };
}

/** A request held until the test answers it. */
struct PendingRequest
{
    std::vector<uint8_t> request;
    std::move_only_function<void(const std::error_code&,
                                 std::span<const uint8_t>)>
        callback;
};

/** Action that holds each request until the test answers it, the way the
 *  requester completes it later from the event loop. */
inline auto deferInto(std::deque<PendingRequest>& pending)
{
    return [&pending](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                      auto callback) {
        pending.push_back(
            {{reqMsg.begin(), reqMsg.end()}, std::move(callback)});
    };
}

} // namespace mock_mctp
