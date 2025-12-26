/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message.hpp>

#include <cstdint>
#include <memory>
#include <string>

constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";

class GpuDevice;
class SmaDevice;
class PcieDevice;

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    mctp::MctpRequester& mctpRequester);

void interfaceRemoved(
    sdbusplus::message_t& message,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices);
