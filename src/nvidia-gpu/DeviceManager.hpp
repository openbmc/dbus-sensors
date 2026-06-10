/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "EndpointState.hpp"
#include "MctpRequester.hpp"
#include "NvidiaGpuDevice.hpp"
#include "NvidiaPcieDevice.hpp"
#include "NvidiaSensorConfig.hpp"
#include "NvidiaSmaDevice.hpp"
#include "Utils.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/system/error_code.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

class DeviceManager
{
  public:
    DeviceManager(boost::asio::io_context& io,
                  sdbusplus::asio::object_server& objectServer,
                  std::shared_ptr<sdbusplus::asio::connection> conn,
                  mctp::MctpRequester& mctpRequester);

    void createSensors();
    // Debounced re-discovery: coalesces bursts of entity-manager
    // configuration changes into a single createSensors().
    void scheduleRescan();
    void onConfigInterfaceRemoved(sdbusplus::message_t& message);
    void onConnectivityChanged(sdbusplus::message_t& msg);

  private:
    void processSensorConfigs(const ManagedObjectType& resp);
    void discoverDevices(const SensorConfigs& configs,
                         const sdbusplus::object_path& entityObjectPath);
    void queryEndpoints(const SensorConfigs& configs,
                        const sdbusplus::object_path& entityObjectPath,
                        const boost::system::error_code& ec,
                        const GetSubTreeType& ret);
    void processEndpoint(const SensorConfigs& configs,
                         const sdbusplus::object_path& entityObjectPath,
                         const sdbusplus::object_path& mctpObjectPath,
                         const boost::system::error_code& ec,
                         const SensorBaseConfigMap& endpoint);
    void queryDeviceIdentification(
        const SensorConfigs& configs,
        const sdbusplus::object_path& entityObjectPath,
        const sdbusplus::object_path& mctpObjectPath, uint8_t eid);
    void processQueryDeviceIdResponse(
        const SensorConfigs& configs,
        const sdbusplus::object_path& entityObjectPath,
        const sdbusplus::object_path& mctpObjectPath, uint8_t eid,
        const std::error_code& sendRecvMsgResult,
        std::span<const uint8_t> queryDeviceIdentificationResponse);

    void applyEvent(const sdbusplus::object_path& mctpObjectPath,
                    EndpointEvent event);

    boost::asio::io_context& io;
    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    mctp::MctpRequester& mctpRequester;

    struct SmaDeviceRecord
    {
        std::shared_ptr<SmaDevice> device;
        std::string name;
        // The mctpd endpoint the device is reached through. A Connectivity
        // signal only carries this path, so it is what a device is found by.
        sdbusplus::object_path mctpObjectPath;
        uint8_t eid{};
        EndpointState state{EndpointState::Init};
    };

    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>
        gpuDevices;
    std::vector<SmaDeviceRecord> smaDevices;
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>
        pcieDevices;

    boost::asio::steady_timer configTimer;
};
