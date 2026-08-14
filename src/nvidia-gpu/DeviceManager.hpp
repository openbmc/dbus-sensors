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
                         const sdbusplus::object_path& mctpObjectPath,
                         const boost::system::error_code& ec,
                         const SensorBaseConfigMap& endpoint);
    void checkAssociationAndQueryDevice(
        const SensorConfigs& configs,
        const sdbusplus::object_path& mctpObjectPath, uint8_t eid);
    void getAssociationEndpoints(const SensorConfigs& configs,
                                 const sdbusplus::object_path& mctpObjectPath,
                                 uint8_t eid,
                                 const sdbusplus::object_path& associationPath,
                                 const std::string& associationService);
    void processAssociationEndpointsResult(
        const SensorConfigs& configs,
        const sdbusplus::object_path& mctpObjectPath, uint8_t eid,
        const boost::system::error_code& ec,
        const std::variant<std::vector<std::string>>& value);
    void getConfigService(const SensorConfigs& configs,
                          const sdbusplus::object_path& mctpObjectPath,
                          uint8_t eid,
                          const sdbusplus::object_path& configPath);
    void getConfigProperties(const SensorConfigs& configs,
                             const sdbusplus::object_path& mctpObjectPath,
                             uint8_t eid,
                             const sdbusplus::object_path& configPath,
                             const std::string& configService);
    void processConfigPropertiesResult(
        const SensorConfigs& configs,
        const sdbusplus::object_path& mctpObjectPath, uint8_t eid,
        const sdbusplus::object_path& configPath,
        const boost::system::error_code& ec,
        const SensorBaseConfigMap& configProps);
    // The name a board's configuration was matched under, which is what a
    // platform record names its board with, is a property on the board rather
    // than something the object path can be derived from. Read every board's
    // once per sweep instead of for every device that resolves against one.
    void collectBoardPaths(std::function<void()> done);
    // Resolve the configuration object a device's sensors should be
    // associated with: the NvidiaMctpVdm configuration under the named board.
    // The resolved path, or fallbackPath if the board or its configuration
    // cannot be found, is handed to done().
    using ConfigPathHandler = std::function<void(const std::string&)>;
    void findBoardInventoryPath(const std::string& boardName,
                                const sdbusplus::object_path& fallbackPath,
                                uint8_t eid, const ConfigPathHandler& done);
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

    // Best-effort read of the endpoint's Common.UUID, cached for identity
    // verification when the endpoint object is removed and re-added.
    void fetchEndpointUuid(const sdbusplus::object_path& mctpObjectPath);
    // Re-attach a previously-offline endpoint after confirming its UUID still
    // matches the device we have at that path.
    void verifyAndReadd(const sdbusplus::object_path& mctpObjectPath);
    // A known device (matched by UUID) reappeared at a new endpoint path
    // because its EID changed: re-bind the existing device object to the new
    // EID/path in place, without rebuilding its D-Bus objects.
    void reattachByUuid(const sdbusplus::object_path& mctpObjectPath);
    void applyEvent(const sdbusplus::object_path& mctpObjectPath,
                    EndpointEvent event);

    // A transient D-Bus failure during the per-endpoint config-resolution
    // chain schedules a bounded number of full-sweep retries before the
    // endpoint is given up on. Returns false once the cap is reached.
    bool retryDiscovery(const sdbusplus::object_path& mctpObjectPath,
                        uint8_t eid);

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
        // The endpoint's Common.UUID, which is what identifies the device
        // again once mctpd has taken the endpoint object away.
        std::string uuid;
        uint8_t eid{};
        EndpointState state{EndpointState::Init};
    };

    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>
        gpuDevices;
    std::vector<SmaDeviceRecord> smaDevices;
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>
        pcieDevices;

    boost::asio::steady_timer configTimer;

    // Board configuration name to the inventory object it was exported at,
    // rebuilt at the start of every discovery sweep.
    boost::container::flat_map<std::string, sdbusplus::object_path> boardPaths;

    // key = mctpd endpoint path -> number of transient-error discovery retries
    // already scheduled for it; capped so a persistently failing endpoint does
    // not re-trigger sweeps forever. Reset when the endpoint's config resolves.
    std::unordered_map<std::string, unsigned> discoveryRetries;
};
