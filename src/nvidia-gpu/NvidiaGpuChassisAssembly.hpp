/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// GPU chassis assembly inventory for Redfish Assembly schema.
// Creates two Panel objects (Device Assembly and Board Assembly) under
// the chassis board path.  bmcweb discovers them via the "containing"
// association and populates GET /redfish/v1/Chassis/{id}/Assembly/.
class ChassisAssembly : public std::enable_shared_from_this<ChassisAssembly>
{
  public:
    ChassisAssembly(sdbusplus::asio::object_server& objectServer,
                    mctp::MctpRequester& mctpRequester,
                    const std::string& gpuName, const std::string& chassisPath,
                    uint8_t eid, boost::asio::io_context& io);

    ~ChassisAssembly();

    void init();

  private:
    struct AssemblyInterfaces
    {
        std::shared_ptr<sdbusplus::asio::dbus_interface> panelIface;
        std::shared_ptr<sdbusplus::asio::dbus_interface> assetIface;
        std::shared_ptr<sdbusplus::asio::dbus_interface> physicalContextIface;
        std::shared_ptr<sdbusplus::asio::dbus_interface> embeddedIface;
        std::shared_ptr<sdbusplus::asio::dbus_interface> itemIface;
        std::shared_ptr<sdbusplus::asio::dbus_interface> operationalStatusIface;
        std::shared_ptr<sdbusplus::asio::dbus_interface> associationIface;
    };

    using PropertyTarget =
        std::pair<std::shared_ptr<sdbusplus::asio::dbus_interface>,
                  std::string>;

    struct PropertyInfo
    {
        std::vector<PropertyTarget> targets;
        int retryCount{0};
        bool isPending{false};
    };

    AssemblyInterfaces createAssemblyInterfaces(const std::string& path,
                                                const std::string& chassisPath);
    void removeAssemblyInterfaces(AssemblyInterfaces& ifaces);
    void addPropertyTarget(
        gpu::InventoryPropertyId propertyId,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>& interface,
        const std::string& propertyName);
    void sendRequest(gpu::InventoryPropertyId propertyId);
    void handleResponse(gpu::InventoryPropertyId propertyId,
                        const std::error_code& ec,
                        std::span<const uint8_t> buffer);
    void processNextProperty();
    std::optional<gpu::InventoryPropertyId> getNextPendingProperty() const;

    sdbusplus::asio::object_server& objectServer;
    mctp::MctpRequester& mctpRequester;
    uint8_t eid;
    boost::asio::steady_timer retryTimer;

    AssemblyInterfaces deviceAssembly;
    AssemblyInterfaces boardAssembly;

    std::unordered_map<gpu::InventoryPropertyId, PropertyInfo> properties;
    std::array<uint8_t, sizeof(gpu::GetInventoryInformationRequest)>
        requestBuffer{};

    static constexpr std::chrono::seconds retryDelay{5};
    static constexpr int maxRetryAttempts = 3;
};
