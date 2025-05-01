/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "Requester.hpp"

#include "MockSocket.hpp"
#include "MockSocketInterface.hpp"
#include "OcpMctpVdm.hpp"

#include <boost/asio/io_context.hpp>

#include <gtest/gtest.h>

using namespace mctp;

using TestId = int;
using CallbackId = int;

class RequesterTest : public ::testing::Test
{
  protected:
    void SetUp() override {}

    void TearDown() override {}

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

    std::unordered_map<TestId, std::unordered_map<CallbackId, bool>> state;
    std::unique_ptr<boost::asio::io_context> ctx;
    std::unique_ptr<Requester<MockSocketInterface, MockSocket>> requester;
};

// Test basic request-response flow
TEST_F(RequesterTest, BasicRequestResponse)
{
    const eid_t eid = 1;
    auto request = createValidRequest();
    const TestId testId = 1;
    const CallbackId callbackId = 1;

    ctx = std::make_unique<boost::asio::io_context>();
    requester =
        std::make_unique<Requester<MockSocketInterface, MockSocket>>(*ctx);

    requester->sendRecvMsg(
        eid, request,
        [this, testId,
         callbackId](int status, const Response& resp [[maybe_unused]]) {
            EXPECT_EQ(status, 0);
            state[testId][callbackId] = true;
            ctx->stop();
        });

    ctx->run();

    EXPECT_TRUE(state[testId][callbackId]);
}

// Test multiple requests to the same endpoint are queued properly
TEST_F(RequesterTest, MultipleRequestsToSameEndpoint)
{
    const eid_t eid = 1;
    const int numRequests = 3;

    const TestId testId = 2;
    std::vector<CallbackId> callbackIds;

    ctx = std::make_unique<boost::asio::io_context>();
    requester =
        std::make_unique<Requester<MockSocketInterface, MockSocket>>(*ctx);

    // Create and send multiple requests
    for (int i = 0; i < numRequests; ++i)
    {
        auto request = createValidRequest(i + 1);

        requester->sendRecvMsg(
            eid, request,
            [this, testId, &callbackIds, callbackId = i + 1](
                int status, const Response& resp [[maybe_unused]]) {
                EXPECT_EQ(status, 0);
                state[testId][callbackId] = true;
                callbackIds.push_back(callbackId);
                if (callbackIds.size() == numRequests)
                {
                    ctx->stop();
                }
            });
    }

    // Run the io_context to process all requests
    ctx->run();

    // Verify callback IDs are in order
    for (int i = 0; i < numRequests; ++i)
    {
        EXPECT_EQ(callbackIds[i], i + 1);
    }

    // Verify all callbacks were called
    for (const auto& callbackId : callbackIds)
    {
        EXPECT_TRUE(state[testId][callbackId]);
    }
}

// Test requests to different endpoints are handled independently
TEST_F(RequesterTest, MultipleEndpoints)
{
    const eid_t eid1 = 1;
    const eid_t eid2 = 2;
    const TestId testId = 3;
    const CallbackId callbackId1 = 1;
    const CallbackId callbackId2 = 2;

    ctx = std::make_unique<boost::asio::io_context>();
    requester =
        std::make_unique<Requester<MockSocketInterface, MockSocket>>(*ctx);

    // Send request to first endpoint
    auto request1 = createValidRequest(1);
    requester->sendRecvMsg(
        eid1, request1,
        [this, testId, callbackId1,
         callbackId2](int status, const Response& resp [[maybe_unused]]) {
            EXPECT_EQ(status, 0);
            state[testId][callbackId1] = true;
            if (state[testId][callbackId2])
            {
                ctx->stop();
            }
        });

    // Send request to second endpoint
    auto request2 = createValidRequest(2);
    requester->sendRecvMsg(
        eid2, request2,
        [this, testId, callbackId2,
         callbackId1](int status, const Response& resp [[maybe_unused]]) {
            EXPECT_EQ(status, 0);
            state[testId][callbackId2] = true;
            if (state[testId][callbackId1])
            {
                ctx->stop();
            }
        });

    ctx->run();

    EXPECT_TRUE(state[testId][callbackId1]);
    EXPECT_TRUE(state[testId][callbackId2]);
}

// Test invalid message size handling
TEST_F(RequesterTest, InvalidMessageSize)
{
    const eid_t eid = 1;
    std::vector<uint8_t> invalidRequest = {0x01}; // Too small
    const TestId testId = 5;
    const CallbackId callbackId = 1;

    ctx = std::make_unique<boost::asio::io_context>();
    requester =
        std::make_unique<Requester<MockSocketInterface, MockSocket>>(*ctx);

    requester->sendRecvMsg(
        eid, invalidRequest,
        [this, testId,
         callbackId](int status, const Response& resp [[maybe_unused]]) {
            EXPECT_EQ(status, -EBADMSG);
            state[testId][callbackId] = true;
            ctx->stop();
        });

    ctx->run();

    EXPECT_TRUE(state[testId][callbackId]);
}

// Test concurrent requests to multiple endpoints
TEST_F(RequesterTest, ConcurrentRequests)
{
    const int numEndpoints = 3;
    const int requestsPerEndpoint = 2;
    const TestId testId = 7;

    ctx = std::make_unique<boost::asio::io_context>();
    requester =
        std::make_unique<Requester<MockSocketInterface, MockSocket>>(*ctx);

    int completedCallbacks = 0;
    const int totalCallbacks = numEndpoints * requestsPerEndpoint;

    // Send multiple requests to each endpoint
    for (int eid = 1; eid <= numEndpoints; ++eid)
    {
        for (int i = 0; i < requestsPerEndpoint; ++i)
        {
            auto request = createValidRequest(i + 1);
            const CallbackId callbackId =
                (eid - 1) * requestsPerEndpoint + i + 1;

            requester->sendRecvMsg(
                eid, request,
                [this, testId, callbackId, &completedCallbacks](
                    int status, const Response& resp [[maybe_unused]]) {
                    EXPECT_EQ(status, 0);
                    state[testId][callbackId] = true;
                    completedCallbacks++;
                    if (completedCallbacks == totalCallbacks)
                    {
                        ctx->stop();
                    }
                });
        }
    }

    ctx->run();

    // Verify all callbacks were called
    for (int i = 1; i <= totalCallbacks; ++i)
    {
        EXPECT_TRUE(state[testId][i]);
    }
}
