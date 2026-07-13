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
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <unordered_map>
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
                         const sdbusplus::object_path& endpointPath,
                         const boost::system::error_code& ec,
                         const SensorBaseConfigMap& endpoint);
    void checkAssociationAndQueryDevice(
        const SensorConfigs& configs,
        const sdbusplus::object_path& endpointPath, uint8_t eid);
    void getAssociationEndpoints(const SensorConfigs& configs,
                                 const sdbusplus::object_path& endpointPath,
                                 uint8_t eid,
                                 const sdbusplus::object_path& associationPath,
                                 const std::string& associationService);
    void processAssociationEndpointsResult(
        const SensorConfigs& configs,
        const sdbusplus::object_path& endpointPath, uint8_t eid,
        const boost::system::error_code& ec,
        const std::variant<std::vector<std::string>>& value);
    void getConfigService(const SensorConfigs& configs,
                          const sdbusplus::object_path& endpointPath,
                          uint8_t eid,
                          const sdbusplus::object_path& configPath);
    void getConfigProperties(const SensorConfigs& configs,
                             const sdbusplus::object_path& endpointPath,
                             uint8_t eid,
                             const sdbusplus::object_path& configPath,
                             const std::string& configService);
    void processConfigPropertiesResult(
        const SensorConfigs& configs,
        const sdbusplus::object_path& endpointPath, uint8_t eid,
        const sdbusplus::object_path& configPath,
        const boost::system::error_code& ec,
        const SensorBaseConfigMap& configProps);
    // The name a board's configuration was matched under, which is what a
    // platform record names its board with, is a property on the board rather
    // than something the object path can be derived from. Read every board's
    // once per sweep instead of for every device that resolves against one.
    void collectBoardPaths(std::function<void()> done);
    void findBoardInventoryPath(const SensorConfigs& configs,
                                const sdbusplus::object_path& endpointPath,
                                uint8_t eid, const std::string& boardName,
                                const sdbusplus::object_path& configPath);
    void processNvidiaMctpVdmConfigSearch(
        const SensorConfigs& configs,
        const sdbusplus::object_path& endpointPath, uint8_t eid,
        const sdbusplus::object_path& inventoryPath,
        const sdbusplus::object_path& configPath,
        const boost::system::error_code& ec, const GetSubTreeType& ret);
    void queryDeviceIdentification(
        const SensorConfigs& configs, const sdbusplus::object_path& path,
        const sdbusplus::object_path& endpointPath, uint8_t eid);
    void processQueryDeviceIdResponse(
        const SensorConfigs& configs, const sdbusplus::object_path& path,
        const sdbusplus::object_path& endpointPath, uint8_t eid,
        const std::error_code& sendRecvMsgResult,
        std::span<const uint8_t> queryDeviceIdentificationResponse);

    void registerEndpoint(const sdbusplus::object_path& endpointPath,
                          uint8_t eid,
                          const std::shared_ptr<DeviceInterface>& device);
    // Best-effort read of the endpoint's Common.UUID, cached for identity
    // verification when the endpoint object is removed and re-added.
    void fetchEndpointUuid(const sdbusplus::object_path& endpointPath);
    // Re-attach a previously-offline endpoint after confirming its UUID still
    // matches the device we have at that path.
    void verifyAndReadd(const sdbusplus::object_path& endpointPath);
    // A known device (matched by UUID) reappeared at a new endpoint path
    // because its EID changed: re-bind the existing device object to the new
    // EID/path in place, without rebuilding its D-Bus objects.
    void reattachByUuid(const sdbusplus::object_path& endpointPath);
    void applyEvent(const sdbusplus::object_path& endpointPath,
                    EndpointEvent event);

    // A transient D-Bus failure during the per-endpoint config-resolution
    // chain schedules a bounded number of full-sweep retries before the
    // endpoint is given up on. Returns false once the cap is reached.
    bool retryDiscovery(const sdbusplus::object_path& endpointPath,
                        uint8_t eid);

    boost::asio::io_context& io;
    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    mctp::MctpRequester& mctpRequester;

    // Board configuration name to the inventory object it was exported at,
    // rebuilt at the start of every discovery sweep.
    boost::container::flat_map<std::string, sdbusplus::object_path> boardPaths;

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
    std::unordered_map<sdbusplus::object_path, EndpointRecord> endpoints;
    // key = endpoint Common.UUID -> device, for matching across EID changes
    std::unordered_map<std::string, std::weak_ptr<DeviceInterface>>
        uuidToDevice;
    // key = mctpd endpoint path -> number of transient-error discovery retries
    // already scheduled for it; capped so a persistently failing endpoint does
    // not re-trigger sweeps forever. Reset when the endpoint's config resolves.
    std::unordered_map<std::string, unsigned> discoveryRetries;
};
