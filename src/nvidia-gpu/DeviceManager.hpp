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
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

// An NvidiaMctpVdm record as EntityManager exposes it. Discovery carries it
// as it is, and it is only read into what a device takes once the kind of
// device it describes is known.
struct MctpVdmRecord
{
    sdbusplus::object_path path;
    SensorBaseConfigMap properties;
};

class DeviceManager
{
  public:
    DeviceManager(boost::asio::io_context& io,
                  sdbusplus::asio::object_server& objectServer,
                  std::shared_ptr<sdbusplus::asio::connection> conn,
                  mctp::MctpRequester& mctpRequester);

    void createSensors();
    // Debounced re-discovery: coalesces bursts of entity-manager
    // configuration changes and endpoints added by mctpd into a single
    // createSensors().
    void scheduleRescan();
    void onConfigInterfaceRemoved(sdbusplus::message_t& message);
    void onConnectivityChanged(sdbusplus::message_t& msg);
    void onEndpointRemoved(sdbusplus::message_t& msg);
    void onEndpointAdded(sdbusplus::message_t& msg);

  private:
    void discoverDevices();
    void queryEndpoints(const boost::system::error_code& ec,
                        const GetSubTreeType& ret);
    void processEndpoint(const sdbusplus::object_path& mctpObjectPath,
                         const boost::system::error_code& ec,
                         const SensorBaseConfigMap& endpoint);
    void checkAssociationAndQueryDevice(
        const sdbusplus::object_path& mctpObjectPath, uint8_t eid);
    void getConfigProperties(const sdbusplus::object_path& mctpObjectPath,
                             uint8_t eid,
                             const sdbusplus::object_path& configPath,
                             const std::string& configService);
    void processConfigPropertiesResult(
        const sdbusplus::object_path& mctpObjectPath, uint8_t eid,
        const sdbusplus::object_path& configPath,
        const boost::system::error_code& ec,
        const SensorBaseConfigMap& configProps);
    // Resolve the configuration object a device's sensors should be
    // associated with: the NvidiaMctpVdm configuration under the named board.
    // The resolved path, or fallbackPath if the board or its configuration
    // cannot be found, is handed to done().
    using ConfigPathHandler = std::function<void(const std::string&)>;
    using BoardPathHandler =
        std::function<void(const std::optional<sdbusplus::object_path>&)>;
    void lookupBoard(const std::string& boardName, BoardPathHandler done);
    void findBoardInventoryPath(const std::string& boardName,
                                const sdbusplus::object_path& fallbackPath,
                                uint8_t eid, const ConfigPathHandler& done);
    void findBoardConfig(const sdbusplus::object_path& inventoryPath,
                         const sdbusplus::object_path& fallbackPath,
                         uint8_t eid, const ConfigPathHandler& done);
    void queryDeviceIdentification(const MctpVdmRecord& record,
                                   const sdbusplus::object_path& mctpObjectPath,
                                   uint8_t eid);
    void processQueryDeviceIdResponse(
        const MctpVdmRecord& record,
        const sdbusplus::object_path& mctpObjectPath, uint8_t eid,
        const std::error_code& sendRecvMsgResult,
        std::span<const uint8_t> queryDeviceIdentificationResponse);

    // Best-effort read of the endpoint's Common.UUID, cached for identity
    // verification when the endpoint object is removed and re-added.
    void fetchEndpointUuid(const sdbusplus::object_path& mctpObjectPath);
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

    // Boards looked up so far in this discovery sweep.
    boost::container::flat_map<std::string, sdbusplus::object_path> boardPaths;
    // Devices waiting on a board lookup that is still in flight.
    std::unordered_map<std::string, std::vector<BoardPathHandler>> boardLookups;

    std::unordered_map<std::string, unsigned> discoveryRetries;
};
