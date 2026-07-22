/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "DeviceInterface.hpp"
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
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

class DeviceManager
{
  public:
    DeviceManager(boost::asio::io_context& io,
                  sdbusplus::asio::object_server& objectServer,
                  std::shared_ptr<sdbusplus::asio::connection> conn,
                  mctp::MctpRequester& mctpRequester);

    void createSensors();
    // Debounced re-discovery: coalesces bursts of entity-manager config
    // changes and mctpd connectivity events into a single createSensors().
    void scheduleRescan();
    void onConfigInterfaceRemoved(sdbusplus::message_t& message);
    void onConnectivityChanged(sdbusplus::message_t& msg);
    // mctpd removed/re-added an endpoint object (device reset / power cycle).
    void onEndpointRemoved(sdbusplus::message_t& msg);
    void onEndpointAdded(sdbusplus::message_t& msg);

  private:
    void discoverDevices(const SensorConfigs& configs);
    void queryEndpoints(const SensorConfigs& configs,
                        const boost::system::error_code& ec,
                        const GetSubTreeType& ret);
    void processEndpoint(const SensorConfigs& configs,
                         const std::string& endpointPath,
                         const boost::system::error_code& ec,
                         const SensorBaseConfigMap& endpoint,
                         std::optional<std::pair<uint8_t, uint8_t>> bridgePool);
    void checkAssociationAndQueryDevice(
        const SensorConfigs& configs, const std::string& endpointPath,
        uint8_t eid, std::optional<std::pair<uint8_t, uint8_t>> bridgePool);
    void getAssociationEndpoints(
        const SensorConfigs& configs, const std::string& endpointPath,
        uint8_t eid, const std::string& associationPath,
        const std::string& associationService,
        std::optional<std::pair<uint8_t, uint8_t>> bridgePool);
    void processAssociationEndpointsResult(
        const SensorConfigs& configs, const std::string& endpointPath,
        uint8_t eid, const boost::system::error_code& ec,
        const std::variant<std::vector<std::string>>& value,
        std::optional<std::pair<uint8_t, uint8_t>> bridgePool);
    void getConfigService(
        const SensorConfigs& configs, const std::string& endpointPath,
        uint8_t eid, const std::string& configPath,
        std::optional<std::pair<uint8_t, uint8_t>> bridgePool);
    void getConfigProperties(
        const SensorConfigs& configs, const std::string& endpointPath,
        uint8_t eid, const std::string& configPath,
        const std::string& configService,
        std::optional<std::pair<uint8_t, uint8_t>> bridgePool);
    void processConfigPropertiesResult(
        const SensorConfigs& configs, const std::string& endpointPath,
        uint8_t eid, const std::string& configPath,
        const boost::system::error_code& ec,
        const SensorBaseConfigMap& configProps,
        std::optional<std::pair<uint8_t, uint8_t>> bridgePool);
    void findBoardInventoryPath(
        const SensorConfigs& configs, const std::string& endpointPath,
        uint8_t eid, const std::string& boardName,
        const std::string& configPath,
        std::optional<std::pair<uint8_t, uint8_t>> bridgePool,
        const std::optional<std::vector<std::string>>& bridgedEndpoints);
    void processBoardInventoryResult(
        const SensorConfigs& configs, const std::string& endpointPath,
        uint8_t eid, const std::string& boardName,
        const std::string& configPath, const boost::system::error_code& ec,
        const GetSubTreeType& ret,
        std::optional<std::pair<uint8_t, uint8_t>> bridgePool,
        const std::optional<std::vector<std::string>>& bridgedEndpoints);
    void processNvidiaMctpVdmConfigSearch(
        const SensorConfigs& configs, const std::string& endpointPath,
        uint8_t eid, const std::string& inventoryPath,
        const std::string& configPath, const boost::system::error_code& ec,
        const GetSubTreeType& ret,
        std::optional<std::pair<uint8_t, uint8_t>> bridgePool,
        const std::optional<std::vector<std::string>>& bridgedEndpoints);
    void queryDevicesForEndpoint(
        const SensorConfigs& configs, const std::string& configPath,
        const std::string& endpointPath, uint8_t eid,
        const std::string& chassisPath,
        const std::optional<std::pair<uint8_t, uint8_t>>& bridgePool,
        const std::optional<std::vector<std::string>>& bridgedEndpoints);
    void queryDeviceIdentification(
        const SensorConfigs& configs, const std::string& path,
        const std::string& endpointPath, uint8_t eid,
        const std::string& chassisPath, const std::string& deviceName);
    void processQueryDeviceIdResponse(
        const SensorConfigs& configs, const std::string& path,
        const std::string& endpointPath, uint8_t eid,
        const std::string& chassisPath, const std::string& deviceName,
        const std::error_code& sendRecvMsgResult,
        std::span<const uint8_t> queryDeviceIdentificationResponse);

    void registerEndpoint(const std::string& endpointPath, uint8_t eid,
                          const std::shared_ptr<DeviceInterface>& device);
    // Best-effort read of the endpoint's Common.UUID, cached for identity
    // verification when the endpoint object is removed and re-added.
    void fetchEndpointUuid(const std::string& endpointPath);
    // Re-attach a previously-offline endpoint after confirming its UUID still
    // matches the device we have at that path.
    void verifyAndReadd(const std::string& endpointPath);
    // A known device (matched by UUID) reappeared at a new endpoint path
    // because its EID changed: re-bind the existing device object to the new
    // EID/path in place, without rebuilding its D-Bus objects.
    void reattachByUuid(const std::string& endpointPath);
    void applyEvent(const std::string& endpointPath, EndpointEvent event);

    // A transient D-Bus failure during the per-endpoint config-resolution
    // chain schedules a bounded number of full-sweep retries before the
    // endpoint is given up on. Returns false once the cap is reached.
    bool retryDiscovery(const std::string& endpointPath, uint8_t eid);

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

    boost::asio::steady_timer configTimer;

    struct EndpointRecord
    {
        std::weak_ptr<DeviceInterface> device; // SmaDevice for now
        uint8_t eid{};
        EndpointState state{EndpointState::Init};
        std::string uuid; // endpoint Common.UUID, for re-add identity check
    };
    // key = mctpd endpoint D-Bus object path
    std::unordered_map<std::string, EndpointRecord> endpoints;
    // key = endpoint Common.UUID -> device, for matching across EID changes
    std::unordered_map<std::string, std::weak_ptr<DeviceInterface>>
        uuidToDevice;
    // key = mctpd endpoint path -> number of transient-error discovery retries
    // already scheduled for it; capped so a persistently failing endpoint does
    // not re-trigger sweeps forever. Reset when the endpoint's config resolves.
    std::unordered_map<std::string, unsigned> discoveryRetries;
};
