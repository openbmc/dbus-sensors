/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "MctpRequester.hpp"

#include <af-mctp.h>
#include <ocp_ami.h>
#include <transport.h>

#include <boost/asio/awaitable.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/experimental/awaitable_operators.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/local/datagram_protocol.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/this_coro.hpp>
#include <boost/asio/use_awaitable.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/exception/exception.hpp>
#include <boost/system/system_error.hpp>
#include <phosphor-logging/lg2.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

using namespace std::literals;

namespace mctp
{
using namespace boost::asio::experimental::awaitable_operators;

MctpRequester::MctpRequester(boost::asio::io_context& ctx, uint8_t msgType,
                             bool verbose) :
    verbose(verbose),
    ctx(ctx), socket(ctx), flag(MSG_PEEK | MSG_TRUNC)
{
    auto rc = mctp_transport_af_mctp_init(&afMctp, msgType);
    if (rc < 0)
    {
        lg2::error("mctp_transport_af_mctp_init() failed");
        return;
    }

    transport = mctp_transport_af_mctp_core(afMctp);

    auto sockfd = mctp_transport_get_fd(transport);

    try
    {
        socket.assign(boost::asio::local::datagram_protocol{}, sockfd);
    }
    catch (const boost::exception& ex)
    {
        std::string info = boost::diagnostic_information(ex);
        lg2::error(
            "MctpRequester failed to connect to the MCTP socket - Error={ER}.",
            "ER", info);
        return;
    }

    socket.non_blocking(true);
}

MctpRequester::~MctpRequester()
{
    mctp_transport_af_mctp_destroy(afMctp);
}

boost::asio::awaitable<std::pair<int, std::vector<uint8_t>>>
    MctpRequester::sendRecvMsg(mctp_eid_t eid,
                               const std::vector<uint8_t>& reqMsg)
{
    std::vector<uint8_t> respMsg{};

    auto rc = mctp_transport_send_msg(transport, eid, reqMsg.data(),
                                      reqMsg.size());

    // Disable clang-tidy check as the error is in header
    // boost/asio/impl/awaitable.hpp and to exclude that header from clang-tidy
    // checks, clang-tidy configuration option ExcludeHeaderFilterRegex is only
    // supported in clang-tidy version 19.

    // NOLINTNEXTLINE
    if (rc < 0)
    {
        lg2::error(
            "MctpRequester failed send data to the MCTP Socket - Error={EC}.",
            "EC", rc);

        co_return std::make_pair(rc, respMsg);
    }

    try
    {
        boost::asio::steady_timer timoutTimer(
            co_await boost::asio::this_coro::executor);

        timoutTimer.expires_after(2s);

        const auto result =
            co_await (socket.async_receive(boost::asio::buffer(respMsg), flag,
                                           boost::asio::use_awaitable) ||
                      timoutTimer.async_wait(boost::asio::use_awaitable));

        if (result.index() == 1)
        {
            // timeout
            co_return std::make_pair(-1, respMsg);
        }
    }
    catch (const boost::system::system_error& e)
    {
        boost::system::error_code ec = e.code();

        lg2::error(
            "MctpRequester failed to receive data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());

        co_return std::make_pair(-2, respMsg);
    }

    mctp_eid_t respEid = 0;
    uint8_t* recvMsg = nullptr;
    size_t recvMsgLen = 0;

    rc = mctp_transport_recv_msg(
        transport, &respEid, reinterpret_cast<void**>(&recvMsg), &recvMsgLen);

    boost::system::error_code ec;

    if (rc < 0)
    {
        lg2::error(
            "MctpRequester failed to receive data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());

        co_return std::make_pair(rc, respMsg);
    }

    if (respEid != eid)
    {
        lg2::error(
            "MctpRequester failed to receive data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
            "EC", ec.value(), "ER", ec.message());

        co_return std::make_pair(-3, respMsg);
    }

    if (recvMsgLen > sizeof(struct ocp_ami_binding_pci_vid))
    {
        const auto* reqHdr =
            reinterpret_cast<const ocp_ami_binding_pci_vid*>(reqMsg.data());
        const auto* respHdr =
            reinterpret_cast<const ocp_ami_binding_pci_vid*>(recvMsg);

        if (reqHdr->instance_id != respHdr->instance_id)
        {
            lg2::error(
                "MctpRequester failed to receive data from the MCTP socket - ErrorCode={EC}, Error={ER}.",
                "EC", ec.value(), "ER", ec.message());

            co_return std::make_pair(-4, respMsg);
        }
    }

    respMsg.insert(respMsg.begin(), recvMsg, recvMsg + recvMsgLen);

    co_return std::make_pair(0, std::move(respMsg));
}
} // namespace mctp
