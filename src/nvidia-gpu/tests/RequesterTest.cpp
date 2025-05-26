/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "MctpRequester.hpp"
#include "MockSocket.hpp"
#include "OcpMctpVdm.hpp"
#include "types.hpp"

#include <boost/asio/io_context.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <numeric>
#include <random>
#include <set>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

using namespace mctp;

using TestId = int;
using CallbackId = int;

std::unordered_map<TestId, std::set<CallbackId>> state;

class RequesterTest : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        ctx = std::make_unique<boost::asio::io_context>();
        requester = std::make_unique<Requester<MockSocket>>(
            *ctx, ocp::accelerator_management::messageType);
    }

    void TearDown() override
    {
        if (ctx)
        {
            ctx->stop();
        }
        ctx.reset();
        requester.reset();
    }

    // Helper function to create a valid request message
    static std::vector<uint8_t> createValidRequest(uint8_t instanceId = 1)
    {
        std::vector<uint8_t> request(
            sizeof(ocp::accelerator_management::BindingPciVid));
        auto* hdr = std::bit_cast<ocp::accelerator_management::BindingPciVid*>(
            request.data());
        hdr->instance_id = instanceId;
        return request;
    }

    std::unique_ptr<boost::asio::io_context> ctx;
    std::unique_ptr<Requester<MockSocket>> requester;
};

// Test basic request-response flow
TEST_F(RequesterTest, BasicRequestResponse)
{
    constexpr eid_t eid = 1;
    constexpr TestId testId = 1;
    constexpr CallbackId callbackId = 0;

    requester->sendRecvMsg(
        eid, createValidRequest(),
        [this, testId, callbackId](int ec, const std::vector<uint8_t>&) {
            EXPECT_EQ(ec, 0);
            state[testId].insert(callbackId);
            ctx->stop();
        });

    ctx->run();

    // Only one callback should be called
    EXPECT_EQ(state[testId].size(), 1);
    // Check that the callback was called
    EXPECT_TRUE(state[testId].contains(callbackId));
}

// Test that multiple requests to the same endpoint are queued and completed
TEST_F(RequesterTest, MultipleRequestsToSameEndpoint)
{
    constexpr eid_t eid = 1;
    constexpr int numRequests = ocp::accelerator_management::instanceMax + 1;
    constexpr TestId testId = 2;

    // 32 (ocp::accelerator_management::instanceMax + 1) Random callback ids
    std::array<int, numRequests> callbackIds{
        11, 123, 12, 13, 18, 29, 30, 31, 32, 19, 20, 21, 22, 24, 25, 26,
        27, 28,  14, 15, 16, 33, 34, 17, 35, 36, 37, 38, 39, 49, 41, 44};
    std::vector<int> completedCallbackIds;

    int done = 0;

    for (int idx = 0; idx < numRequests; ++idx)
    {
        requester->sendRecvMsg(
            eid, createValidRequest(static_cast<uint8_t>(idx)),
            [this, testId, callbackId = callbackIds[idx], &done,
             &completedCallbackIds](int ec, const std::vector<uint8_t>&) {
                EXPECT_EQ(ec, 0);
                state[testId].insert(callbackId);
                completedCallbackIds.push_back(callbackId);

                if (++done == numRequests)
                {
                    ctx->stop();
                }
            });
    }

    ctx->run();

    // Check that all callbacks were called
    EXPECT_EQ(state[testId].size(), numRequests);

    // Check that the callbacks were called in the correct order
    // Also, check that the state map contains all the callback ids
    for (int idx = 0; idx < numRequests; ++idx)
    {
        EXPECT_EQ(completedCallbackIds[idx], callbackIds[idx]);
        EXPECT_TRUE(state[testId].contains(callbackIds[idx]));
    }
}

// Test independent handling of two separate endpoints
TEST_F(RequesterTest, MultipleEndpoints)
{
    constexpr eid_t eid1 = 1;
    constexpr eid_t eid2 = 2;
    constexpr TestId testId = 3;

    bool flag1 = false;
    bool flag2 = false;

    auto stop = [this, &flag1, &flag2] {
        if (flag1 && flag2)
        {
            ctx->stop();
        }
    };

    requester->sendRecvMsg(
        eid1, createValidRequest(0),
        [testId, &flag1, &stop](int ec, const std::vector<uint8_t>&) {
            EXPECT_EQ(ec, 0);
            flag1 = true;
            state[testId].insert(0);
            stop();
        });

    requester->sendRecvMsg(
        eid2, createValidRequest(1),
        [testId, &flag2, &stop](int ec, const std::vector<uint8_t>&) {
            EXPECT_EQ(ec, 0);
            flag2 = true;
            state[testId].insert(1);
            stop();
        });

    ctx->run();

    EXPECT_TRUE(flag1);
    EXPECT_TRUE(flag2);
    EXPECT_TRUE(state[testId].contains(0));
    EXPECT_TRUE(state[testId].contains(1));
}

// Test handling of an invalid (too-small) request payload
TEST_F(RequesterTest, InvalidMessageSize)
{
    constexpr eid_t eid = 1;
    constexpr TestId testId = 4;
    constexpr CallbackId callbackId = 0;
    const std::vector<uint8_t> invalidRequest = {0x01}; // clearly too small

    requester->sendRecvMsg(
        eid, invalidRequest,
        [this, testId, callbackId](int ec, const std::vector<uint8_t>&) {
            EXPECT_EQ(ec, -EBADMSG);
            state[testId].insert(callbackId);
            ctx->stop();
        });

    ctx->run();
    EXPECT_TRUE(state[testId].contains(callbackId));
}

// Test concurrent requests to multiple endpoints
/*
 * This test verifies that requests to multiple endpoints are handled correctly
 * and maintain their order within each endpoint. Here's an example:
 *
 * Let's say we have 2 endpoints and 3 requests per endpoint:
 *
 * Initial callback IDs: [0,1,2,3,4,5]
 * After randomization: [5,2,0,4,1,3]
 *
 * Distribution to endpoints:
 * Endpoint 1: [5,2,0]  // First 3 IDs
 * Endpoint 2: [4,1,3]  // Last 3 IDs
 *
 * The test verifies that:
 * 1. All callbacks are executed
 * 2. For each endpoint, callbacks are completed in the same order they were
 * sent e.g., Endpoint 1 must complete callbacks in order [5,2,0] Endpoint 2
 * must complete callbacks in order [4,1,3]
 */
TEST_F(RequesterTest, ConcurrentRequests)
{
    constexpr int numEndpoints = 12;
    constexpr int requestsPerEndpoint =
        ocp::accelerator_management::instanceMax + 1;
    constexpr int totalRequests = numEndpoints * requestsPerEndpoint;
    const TestId testId = 5;

    // Create and randomize callback IDs
    std::vector<int> callbackIds(totalRequests);
    std::ranges::iota(callbackIds.begin(), callbackIds.end(), 0);
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(callbackIds.begin(), callbackIds.end(), g);

    // Map to store callback IDs for each endpoint
    std::map<eid_t, std::vector<int>> endpointCallbacks;
    for (int i = 0; i < totalRequests; ++i)
    {
        eid_t eid = (i / requestsPerEndpoint) + 1;
        endpointCallbacks[eid].push_back(callbackIds[i]);
    }

    // Map to track completed callbacks per endpoint
    std::map<eid_t, std::vector<int>> completedCallbacks;
    int completed = 0;

    // Dispatch requests to each endpoint
    for (const auto& [eid, callbacks] : endpointCallbacks)
    {
        for (size_t idx = 0; idx < callbacks.size(); ++idx)
        {
            auto request = createValidRequest(static_cast<uint8_t>(idx));
            const CallbackId callbackId = callbacks[idx];

            requester->sendRecvMsg(
                eid, request,
                [this, testId, callbackId, &completed, &completedCallbacks,
                 eid](int ec,
                      [[maybe_unused]] const std::vector<uint8_t>& resp) {
                    EXPECT_EQ(ec, 0);
                    state[testId].insert(callbackId);
                    completedCallbacks[eid].push_back(callbackId);

                    if (++completed == totalRequests)
                    {
                        ctx->stop();
                    }
                });
        }
    }

    // Run until all callbacks complete
    ctx->run();

    // Ensure all callbacks were executed
    EXPECT_EQ(completed, totalRequests);

    // Verify that callbacks for each endpoint were handled in order
    for (const auto& [eid, generatedCallbacksForEid] : endpointCallbacks)
    {
        const auto& completedCallbacksForEid = completedCallbacks[eid];

        // Verify all callbacks were executed
        for (const auto& callbackId : generatedCallbacksForEid)
        {
            EXPECT_TRUE(state[testId].contains(callbackId));
        }

        // Verify the order matches
        EXPECT_EQ(generatedCallbacksForEid.size(),
                  completedCallbacksForEid.size());
        for (size_t i = 0; i < generatedCallbacksForEid.size(); ++i)
        {
            EXPECT_EQ(generatedCallbacksForEid[i], completedCallbacksForEid[i])
                << "Callback order mismatch at index " << i << " for endpoint "
                << eid;
        }
    }
}
