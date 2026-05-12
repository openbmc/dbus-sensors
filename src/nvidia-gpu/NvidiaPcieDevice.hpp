/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaPcieFunction.hpp"
#include "NvidiaPcieInterface.hpp"

#include <NvidiaEthPort.hpp>
#include <NvidiaPciePort.hpp>
#include <NvidiaPciePortMetrics.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

constexpr const char* pcieDevicePathPrefix = "/xyz/openbmc_project/inventory/";

constexpr const char* nicPathPrefix = "/xyz/openbmc_project/inventory/";

struct PcieDeviceInfo
{
    uint16_t numUpstreamPorts{};
    std::vector<uint8_t> numDownstreamPorts;
};

class PcieDevice : public std::enable_shared_from_this<PcieDevice>
{
  public:
    PcieDevice(const SensorConfigs& configs, const std::string& name,
               const std::string& path,
               const std::shared_ptr<sdbusplus::asio::connection>& conn,
               uint8_t eid, boost::asio::io_context& io,
               mctp::MctpRequester& mctpRequester,
               sdbusplus::asio::object_server& objectServer);

    const std::string& getPath() const
    {
        return path;
    }

    void init();

  private:
    void makeSensors();

    void read();

    void getPciePortCounts();

    void processPciePortCountsResponse(const std::error_code& ec,
                                       std::span<const uint8_t> response);

    void getNetworkPortAddresses(uint16_t portNumber);

    void processGetNetworkPortAddressesResponse(
        uint16_t portNumber, const std::error_code& ec,
        std::span<const uint8_t> response);

    struct AssetPropertyInfo
    {
        std::string propertyName;
        int retryCount{0};
        bool isPending{false};
    };

    void registerAssetProperty(gpu::InventoryPropertyId propertyId,
                               const std::string& propertyName);

    void processNextAssetProperty();
    void sendAssetPropertyRequest(gpu::InventoryPropertyId propertyId);
    void handleAssetPropertyResponse(gpu::InventoryPropertyId propertyId,
                                     const std::error_code& ec,
                                     std::span<const uint8_t> buffer);
    std::optional<gpu::InventoryPropertyId> getNextPendingAssetProperty() const;

    PcieDeviceInfo pcieDeviceInfo;

    uint8_t eid{};

    std::chrono::milliseconds sensorPollMs;

    boost::asio::steady_timer waitTimer;

    mctp::MctpRequester& mctpRequester;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    sdbusplus::asio::object_server& objectServer;

    SensorConfigs configs;

    std::string name;

    std::string path;

    std::array<uint8_t, ocp::accelerator_management::commonRequestSize>
        getPciePortCountsRequest{};

    std::array<uint8_t, gpu::getPortNetworkAddressesRequestSize>
        getPortNetworkAddressesRequest{};

    std::shared_ptr<NvidiaPcieInterface> pcieInterface;
    std::shared_ptr<NvidiaPcieFunction> pcieFunction;

    std::vector<std::shared_ptr<NvidiaPciePortInfo>> pciePorts;
    std::vector<std::shared_ptr<NvidiaPciePortMetrics>> pciePortMetrics;

    std::shared_ptr<sdbusplus::asio::dbus_interface> networkAdapterInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface>
        networkAdapterAssociationInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> locationCodeInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> embeddedConnectorInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> assetInterface;

    boost::asio::steady_timer assetRetryTimer;
    std::unordered_map<gpu::InventoryPropertyId, AssetPropertyInfo>
        assetProperties;
    std::array<uint8_t, gpu::getInventoryInformationRequestSize>
        assetRequestBuffer{};
    static constexpr std::chrono::seconds assetRetryDelay{5};
    static constexpr int assetMaxRetryAttempts = 3;

    std::vector<std::shared_ptr<NvidiaEthPortMetrics>> ethPortMetrics;
};
