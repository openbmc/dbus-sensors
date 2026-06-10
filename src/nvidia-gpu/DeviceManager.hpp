/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "DeviceInterface.hpp"
#include "EndpointState.hpp"
#include "MctpRequester.hpp"
#include "NvidiaSensorConfig.hpp"
#include "Utils.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/system/error_code.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

class GpuDevice;
class SmaDevice;
class PcieDevice;

class DeviceManager
{
  public:
    DeviceManager(boost::asio::io_context& io,
                  sdbusplus::asio::object_server& objectServer,
                  std::shared_ptr<sdbusplus::asio::connection> conn,
                  mctp::MctpRequester& mctpRequester);

    void start();

  private:
    void createSensors();
    // Debounced re-discovery: coalesces bursts of entity-manager config
    // changes and mctpd InterfacesAdded signals into a single createSensors().
    void scheduleRescan();
    void processSensorConfigs(const ManagedObjectType& resp);
    void discoverDevices(const SensorConfigs& configs, const std::string& path);
    void queryEndpoints(const SensorConfigs& configs, const std::string& path,
                        const boost::system::error_code& ec,
                        const GetSubTreeType& ret);
    void processEndpoint(const SensorConfigs& configs, const std::string& path,
                         const std::string& endpointPath,
                         const boost::system::error_code& ec,
                         const SensorBaseConfigMap& endpoint);
    void queryDeviceIdentification(
        const SensorConfigs& configs, const std::string& path,
        const std::string& endpointPath, uint8_t eid);
    void processQueryDeviceIdResponse(
        const SensorConfigs& configs, const std::string& path,
        const std::string& endpointPath, uint8_t eid,
        const std::error_code& sendRecvMsgResult,
        std::span<const uint8_t> queryDeviceIdentificationResponse);
    void onConfigInterfaceRemoved(sdbusplus::message_t& message);

    void registerEndpoint(const std::string& endpointPath, uint8_t eid,
                          const std::shared_ptr<DeviceInterface>& device);
    void onConnectivityChanged(sdbusplus::message_t& msg);
    void applyEvent(const std::string& endpointPath, EndpointEvent event);

    boost::asio::io_context& io;
    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    mctp::MctpRequester& mctpRequester;

    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>
        gpuDevices;
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>
        smaDevices;
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>
        pcieDevices;

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> configMatches;
    std::unique_ptr<sdbusplus::bus::match_t> configIfaceRemovedMatch;
    boost::asio::steady_timer configTimer;

    struct EndpointRecord
    {
        std::weak_ptr<DeviceInterface> device; // SmaDevice for now
        uint8_t eid{};
        EndpointState state{EndpointState::Init};
    };
    // key = mctpd endpoint D-Bus object path
    std::unordered_map<std::string, EndpointRecord> endpoints;

    std::unique_ptr<sdbusplus::bus::match_t> mctpConnectivityMatch;
};
