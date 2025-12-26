/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaPcieInterface.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

constexpr const char* pcieDevicePathPrefix = "/xyz/openbmc_project/inventory/";

class PcieDevice
{
  public:
    PcieDevice(uint64_t pollRate, const std::string& name,
               const std::shared_ptr<sdbusplus::asio::connection>& conn,
               uint8_t eid, boost::asio::io_context& io,
               mctp::MctpRequester& mctpRequester,
               sdbusplus::asio::object_server& objectServer);

    void init();

  private:
    void makeSensors();

    void read();

    uint8_t eid{};

    std::chrono::milliseconds sensorPollMs;

    boost::asio::steady_timer waitTimer;

    mctp::MctpRequester& mctpRequester;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    sdbusplus::asio::object_server& objectServer;

    std::string name;

    std::shared_ptr<NvidiaPcieInterface> pcieInterface;
};
