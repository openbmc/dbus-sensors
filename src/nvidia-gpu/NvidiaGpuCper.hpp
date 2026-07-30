/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "NvidiaEventReporting.hpp"
#include "NvidiaGpuMctpVdm.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/system/error_code.hpp>
#include <sdbusplus/asio/connection.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_set>
#include <vector>

class NvidiaCperEventHandler :
    public std::enable_shared_from_this<NvidiaCperEventHandler>
{
  public:
    NvidiaCperEventHandler(
        uint8_t eid, boost::asio::io_context& io,
        mctp::MctpRequester& requester,
        const std::shared_ptr<sdbusplus::asio::connection>& conn);

    void handleCperEvent(const EventInfo& eventInfo,
                         std::span<const uint8_t> eventData);

  private:
    static constexpr size_t maxEventHandles = 255;
    static constexpr uint8_t maxGetAttempts = 2;
    static constexpr uint8_t maxAckAttempts = 2;

    void startNextRecord();
    void scheduleRequest();
    void sendRequest();
    void handleResponse(const std::error_code& ec,
                        std::span<const uint8_t> response);
    void handleGetResponse(std::span<const uint8_t> response);
    void handleAckResponse(std::span<const uint8_t> response);
    void handleGetFailure(const std::string& reason);
    void handleAckFailure(const std::string& reason);
    bool appendRecordData(std::span<const uint8_t> data);
    void beginAcknowledgement(bool logRecord);
    void finishRecord();
    void logRecord();
    void handleLoggingComplete(const boost::system::error_code& ec);

    uint8_t eid;
    boost::asio::steady_timer requestTimer;
    mctp::MctpRequester& requester;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    std::deque<uint16_t> eventHandles;
    bool collectionInProgress{};
    gpu::EventLogRecordV2Mode mode{gpu::EventLogRecordV2Mode::GET_DATA};
    uint16_t requestedEventHandle{};
    uint16_t recordEventHandle{};
    uint16_t transferHandle{};
    uint8_t getAttempts{};
    uint8_t ackAttempts{};
    bool shouldLogRecord{};
    std::vector<uint8_t> recordData;
    std::unordered_set<uint16_t> transferHandles;
    std::optional<gpu::CperRecordInfo> recordInfo;
    std::array<uint8_t, gpu::getEventLogRecordV2RequestSize> request{};
};
