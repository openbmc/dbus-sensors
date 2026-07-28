/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <MctpRequester.hpp>
#include <NvidiaGpuLongRunningCommand.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <SerialQueue.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>

struct NvidiaGpuEccMode : std::enable_shared_from_this<NvidiaGpuEccMode>
{
  public:
    NvidiaGpuEccMode(mctp::MctpRequester& mctpRequester,
                     sdbusplus::asio::object_server& objectServer,
                     const std::string& deviceName, uint8_t eid,
                     std::shared_ptr<SerialQueue> longRunningQueue,
                     std::shared_ptr<NvidiaLongRunningResponseHandler>
                         longRunningResponseHandler);

    ~NvidiaGpuEccMode();

    NvidiaGpuEccMode(const NvidiaGpuEccMode&) = delete;
    NvidiaGpuEccMode& operator=(const NvidiaGpuEccMode&) = delete;
    NvidiaGpuEccMode(NvidiaGpuEccMode&&) = delete;
    NvidiaGpuEccMode& operator=(NvidiaGpuEccMode&&) = delete;

    void update();

  private:
    void onGetImmediateSuccess(std::span<const uint8_t> fullBuffer);

    void onGetLongRunningPayload(std::span<const uint8_t> payload);

    void applyEccModeToDbus(bool active, bool enabled);

    int handleEnabledSet(const bool& newEnable, bool& current);

    void sendSetEccModeRequest(bool enable);
    void finishSet(bool succeeded);

    mctp::MctpRequester& mctpRequester;
    uint8_t eid;
    std::shared_ptr<SerialQueue> longRunningQueue;
    std::shared_ptr<NvidiaLongRunningResponseHandler>
        longRunningResponseHandler;

    bool eccModeEnabled{false};

    sdbusplus::asio::object_server& objectServer;

    bool setEccModeInflight{false};

    std::shared_ptr<sdbusplus::asio::dbus_interface> eccModeInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface>
        eccModeAssociationInterface;

    std::shared_ptr<NvidiaGpuLongRunningCommand> getCmd;
    std::shared_ptr<NvidiaGpuLongRunningCommand> setCmd;
};
