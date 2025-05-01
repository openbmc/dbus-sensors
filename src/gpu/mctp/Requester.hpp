/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "KernelSocketInterface.hpp"
#include "MctpSocket.hpp"
#include "OcpMctpVdm.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/local/datagram_protocol.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <phosphor-logging/lg2.hpp>

#include <cstdint>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

namespace mctp
{
/**
 * @brief MCTP requester class
 *
 * This class provides a simple interface for sending and receiving MCTP
 * messages.
 */
template <Socket S>
class Requester
{
    /** @brief Queue of requests */
    using RequestQueue = std::queue<std::unique_ptr<OcpVdmRequest>>;

  public:
    Requester() = delete;

    Requester(const Requester&) = delete;

    Requester(Requester&&) = delete;

    Requester& operator=(const Requester&) = delete;

    Requester& operator=(Requester&&) = delete;

    /**
     * @brief Constructor
     * @param ctx - The IO context to use
     */
    explicit Requester(boost::asio::io_context& ctx) : ctx(ctx), sock(ctx) {}

    /**
     * @brief Send an MCTP request message and receive the response
     *
     * This function sends a request message to the specified endpoint ID and
     * asynchronously waits for a response. It uses the MCTP socket to handle
     * the communication. Results are provided via the callback.
     *
     * @note It's not guaranteed that the request will be sent out immediately.
     * The request will be queued and processed in the order of enqueue.
     *
     * @param[in] eid - The endpoint ID to send the message to
     * @param[in] reqMsg - The request message to send
     * @param[in] callback - Callback function to be invoked when response is
     * received The callback takes two parameters:
     *            - An integer status code (0 for success, negative for error)
     *            - A vector containing the response message bytes
     */
    void sendRecvMsg(const eid_t eid, std::vector<uint8_t> reqMsg,
                     ResponseCallback callback)
    {
        std::error_code err;

        auto req = std::make_unique<OcpVdmRequest>(err, std::move(reqMsg),
                                                   std::move(callback));

        if (err)
        {
            lg2::error("Requester: Error while creating request: {ERR}", "ERR",
                       err.message());
            req->callback(-err.value(), Response{});
            return;
        }

        enqueueRequest(eid, std::move(req));
        processPendingRequests(eid);
    }

  private:
    /**
     * @brief Await response
     *
     * This function waits for any MCTP response to any sent request and
     * invokes the callback with the appropriate status code and response data.
     */
    void awaitResponse()
    {
        sock.recv([this](const boost::system::error_code& err, const eid_t& eid,
                         std::unique_ptr<Response> resp) {
            // Process pending requests once we are done with processing the
            // current response
            boost::asio::post(ctx, [this]() { processPendingRequests(); });

            if (err)
            {
                return;
            }

            if (!resp)
            {
                lg2::error("Requester: Failed to receive message");
                return;
            }

            if (resp->size() <
                sizeof(ocp::accelerator_management::BindingPciVid))
            {
                lg2::error(
                    "Requester: Received Message of Invalid length - {LEN}",
                    "LEN", resp->size());
                return;
            }

            handleResponse(eid, std::move(resp));
        });
    }

    /** @brief Enqueue a request
     *
     * @param[in] eid - The endpoint ID to send the message to.
     * @param[in] req - The request to enqueue.
     */
    void enqueueRequest(const eid_t eid, std::unique_ptr<OcpVdmRequest> req)
    {
        pendingRequests[eid].emplace(std::move(req));
    }

    /** @brief Process pending requests for all endpoints
     *
     * This function processes pending requests for all endpoints.
     */
    void processPendingRequests()
    {
        for (const auto& [eid, _] : pendingRequests)
        {
            processPendingRequests(eid);
        }
    }

    /** @brief Process pending requests
     *
     * This function processes pending requests for a given endpoint ID. It
     * retrieves the next request from the queue and sends it to the endpoint.
     *
     * @param[in] eid - The endpoint ID to process requests for
     */
    void processPendingRequests(const eid_t eid)
    {
        using namespace std::chrono_literals;

        if (hasActiveRequest(eid) || !hasPendingRequests(eid))
        {
            return;
        }

        std::unique_ptr<OcpVdmRequest> req =
            std::move(pendingRequests[eid].front());
        pendingRequests[eid].pop();

        // Send message
        if (!sock.send(eid, *req))
        {
            lg2::error("Requester: Failed to send the data to the MCTP Socket");
            req->callback(-EIO, Response{});
            boost::asio::post(ctx,
                              [this, eid]() { processPendingRequests(eid); });
            return;
        }

        // Set up async receive with timeout
        auto timer = std::make_unique<boost::asio::steady_timer>(ctx);

        timer->expires_after(2s);
        timer->async_wait([callback = req->callback, eid,
                           this](const boost::system::error_code& ec) {
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("EC: {EC}", "EC", ec.message());
                lg2::error("There was a timeout for EID={EID}", "EID", eid);
                callback(-ec.value(), Response{});
                cleanupActiveRequest(eid);
                processPendingRequests(eid);
            }
        }); // Set up handler for when the timer expires

        req->timer = std::move(timer);
        activeRequests[eid] = std::move(req);

        awaitResponse();
    }

    /** @brief Helper function to check if there are pending requests
     *
     * This function checks if there are pending requests for a given endpoint
     * ID.
     *
     * @param[in] eid - The endpoint ID to check for pending requests
     * @return True if there are pending requests, false otherwise
     */
    bool hasPendingRequests(const eid_t eid) const
    {
        return pendingRequests.contains(eid) &&
               !pendingRequests.at(eid).empty();
    }

    /** @brief Helper function to check if there is an active request
     *
     * This function checks if there is an active request for a given endpoint
     * ID.
     *
     * @param[in] eid - The endpoint ID to check for active request
     * @return True if there is an active request, false otherwise
     */
    bool hasActiveRequest(const eid_t eid) const
    {
        return (activeRequests.contains(eid) &&
                (activeRequests.at(eid) != nullptr));
    }

    /** @brief Cancel an active request
     *
     * This function cancels an active request for a given endpoint ID.
     *
     * @param[in] eid - The endpoint ID to cancel the request for
     */
    void cleanupActiveRequest(const eid_t eid)
    {
        activeRequests.erase(eid);
    }

    /** @brief Handle a response
     *
     * This function handles a response for a given endpoint ID.
     *
     * @param[in] eid - The endpoint ID to handle the response for
     * @param[in] response - The response to handle
     */
    void handleResponse(const eid_t eid, std::unique_ptr<Response> response)
    {
        if (!hasActiveRequest(eid))
        {
            lg2::error("Requester: No active request found for EID={EID}, "
                       "Either this is a response for a timed out request"
                       "or we received a response for a request that was not"
                       "sent out by us",
                       "EID", eid);
            return;
        }

        std::unique_ptr<OcpVdmRequest> req = std::move(activeRequests[eid]);

        // Cancel the timer
        req->timer->cancel();
        cleanupActiveRequest(eid);

        const auto* respHdr =
            std::bit_cast<const ocp::accelerator_management::BindingPciVid*>(
                response->data());

        if (req->instanceId != respHdr->instance_id)
        {
            lg2::error(
                "Requester: Instance ID mismatch for EID={EID} - expected={IID}, received={RIID}",
                "EID", eid, "IID", req->instanceId, "RIID",
                static_cast<uint8_t>(respHdr->instance_id));

            req->callback(-EPROTO, Response{});
            return;
        }

        req->callback(0, *response);
    }

    /** @brief IO context to use */
    boost::asio::io_context& ctx;

    /** @brief Pending requests */
    std::unordered_map<eid_t, RequestQueue> pendingRequests;

    /** @brief Active requests. */
    std::unordered_map<eid_t, std::unique_ptr<OcpVdmRequest>> activeRequests;

    /** @brief Local socket */
    S sock;
};

using MctpRequester = Requester<MctpSocket<KernelSocketInterface>>;
} // namespace mctp
