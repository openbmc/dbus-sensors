/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "NvidiaEventReporting.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container_hash/hash.hpp>
#include <boost/system/error_code.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <tuple>
#include <unordered_map>
#include <utility>

class NvidiaLongRunningResponseHandler :
    public std::enable_shared_from_this<NvidiaLongRunningResponseHandler>
{
    using ResponseHandler = std::move_only_function<void(
        boost::system::error_code ec,
        ocp::accelerator_management::CompletionCode cc, uint16_t reasonCode,
        std::span<const uint8_t> responseData)>;

    static constexpr uint8_t longRunningResponseEventClass = 128;
    static constexpr std::chrono::seconds longRunningResponseTimeout{2};

  public:
    explicit NvidiaLongRunningResponseHandler(boost::asio::io_context& io);

    int registerResponseHandler(gpu::MessageType messageType,
                                uint8_t commandCode, uint8_t instanceId,
                                ResponseHandler handler);

    void handler(const EventInfo& eventInfo,
                 std::span<const uint8_t> eventData);

  private:
    using ResponseKey = std::tuple<gpu::MessageType, uint8_t /* commandCode */,
                                   uint8_t /* instanceId */>;

    struct Entry
    {
        ResponseHandler handler;
        boost::asio::steady_timer timer;

        Entry(ResponseHandler handler, boost::asio::io_context& io) :
            handler(std::move(handler)), timer(io)
        {}
    };

    void onTimeout(ResponseKey key);

    boost::asio::io_context& io;
    std::unordered_map<ResponseKey, Entry, boost::hash<ResponseKey>> entries;
};
