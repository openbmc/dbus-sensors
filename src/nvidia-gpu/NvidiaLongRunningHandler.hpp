/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "NvidiaEventReporting.hpp"

#include <boost/container_hash/hash.hpp>

#include <cstdint>
#include <span>
#include <tuple>
#include <unordered_map>

class NvidiaLongRunningResponseHandler
{
    using ResponseHandler = std::function<void(
        ocp::accelerator_management::CompletionCode /*cc*/,
        uint16_t /*reasonCode*/, std::span<const uint8_t> /*buffer*/)>;

    static constexpr uint8_t longRunningResponseEventClass = 128;

  public:
    int registerResponseHandler(gpu::MessageType messageType,
                                uint8_t commandCode,
                                const ResponseHandler& handler);

    void handler(const EventInfo& eventInfo,
                 std::span<const uint8_t> eventData);

  private:
    using ResponseKey = std::tuple<gpu::MessageType, uint8_t>;

    std::unordered_map<ResponseKey, ResponseHandler, boost::hash<ResponseKey>>
        responseHandlers;
};
