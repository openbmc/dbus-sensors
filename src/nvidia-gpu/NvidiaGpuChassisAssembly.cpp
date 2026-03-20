/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuChassisAssembly.hpp"

#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>

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

static constexpr const char* panelIfaceName =
    "xyz.openbmc_project.Inventory.Item.Panel";
static constexpr const char* assetIfaceName =
    "xyz.openbmc_project.Inventory.Decorator.Asset";
static constexpr const char* physicalContextIfaceName =
    "xyz.openbmc_project.Common.PhysicalContext";
static constexpr const char* embeddedIfaceName =
    "xyz.openbmc_project.Inventory.Connector.Embedded";
static constexpr const char* itemIfaceName =
    "xyz.openbmc_project.Inventory.Item";
static constexpr const char* operationalStatusIfaceName =
    "xyz.openbmc_project.State.Decorator.OperationalStatus";

ChassisAssembly::ChassisAssembly(
    sdbusplus::asio::object_server& objectServer,
    mctp::MctpRequester& mctpRequester, const std::string& gpuName,
    const std::string& chassisPath, const uint8_t eid,
    boost::asio::io_context& io) :
    objectServer(objectServer), mctpRequester(mctpRequester), eid(eid),
    retryTimer(io)
{
    const std::string devicePath =
        chassisPath + "/" + gpuName + "_Device_Assembly";
    const std::string boardPath =
        chassisPath + "/" + gpuName + "_Board_Assembly";

    deviceAssembly = createAssemblyInterfaces(devicePath, chassisPath);
    boardAssembly = createAssemblyInterfaces(boardPath, chassisPath);

    // Device PartNumber from DEVICE_PART_NUMBER, Board from BOARD_PART_NUMBER
    addPropertyTarget(gpu::InventoryPropertyId::DEVICE_PART_NUMBER,
                      deviceAssembly.assetIface, "PartNumber");
    addPropertyTarget(gpu::InventoryPropertyId::BOARD_PART_NUMBER,
                      boardAssembly.assetIface, "PartNumber");

    // Shared properties: same MCTP query, result written to both assemblies
    addPropertyTarget(gpu::InventoryPropertyId::SERIAL_NUMBER,
                      deviceAssembly.assetIface, "SerialNumber");
    addPropertyTarget(gpu::InventoryPropertyId::SERIAL_NUMBER,
                      boardAssembly.assetIface, "SerialNumber");

    addPropertyTarget(gpu::InventoryPropertyId::MARKETING_NAME,
                      deviceAssembly.assetIface, "Model");
    addPropertyTarget(gpu::InventoryPropertyId::MARKETING_NAME,
                      boardAssembly.assetIface, "Model");

    addPropertyTarget(gpu::InventoryPropertyId::BUILD_DATE,
                      deviceAssembly.assetIface, "BuildDate");
    addPropertyTarget(gpu::InventoryPropertyId::BUILD_DATE,
                      boardAssembly.assetIface, "BuildDate");
}

ChassisAssembly::AssemblyInterfaces ChassisAssembly::createAssemblyInterfaces(
    const std::string& path, const std::string& chassisPath)
{
    AssemblyInterfaces ifaces;

    ifaces.panelIface = objectServer.add_interface(path, panelIfaceName);
    ifaces.panelIface->initialize();

    ifaces.assetIface = objectServer.add_interface(path, assetIfaceName);
    ifaces.assetIface->register_property("Manufacturer", std::string("NVIDIA"));
    ifaces.assetIface->register_property("PartNumber", std::string{});
    ifaces.assetIface->register_property("SerialNumber", std::string{});
    ifaces.assetIface->register_property("Model", std::string{});
    ifaces.assetIface->register_property("BuildDate", std::string{});
    ifaces.assetIface->initialize();

    ifaces.physicalContextIface =
        objectServer.add_interface(path, physicalContextIfaceName);
    ifaces.physicalContextIface->register_property(
        "Type", std::string("xyz.openbmc_project.Common.PhysicalContext."
                            "PhysicalContextType.Accelerator"));
    ifaces.physicalContextIface->initialize();

    ifaces.embeddedIface = objectServer.add_interface(path, embeddedIfaceName);
    ifaces.embeddedIface->initialize();

    ifaces.itemIface = objectServer.add_interface(path, itemIfaceName);
    ifaces.itemIface->register_property("Present", true);
    ifaces.itemIface->initialize();

    ifaces.operationalStatusIface =
        objectServer.add_interface(path, operationalStatusIfaceName);
    ifaces.operationalStatusIface->register_property("Functional", true);
    ifaces.operationalStatusIface->initialize();

    std::vector<Association> associations;
    associations.emplace_back("contained_by", "containing", chassisPath);
    ifaces.associationIface =
        objectServer.add_interface(path, association::interface);
    ifaces.associationIface->register_property("Associations", associations);
    ifaces.associationIface->initialize();

    return ifaces;
}

void ChassisAssembly::removeAssemblyInterfaces(AssemblyInterfaces& ifaces)
{
    objectServer.remove_interface(ifaces.panelIface);
    objectServer.remove_interface(ifaces.assetIface);
    objectServer.remove_interface(ifaces.physicalContextIface);
    objectServer.remove_interface(ifaces.embeddedIface);
    objectServer.remove_interface(ifaces.itemIface);
    objectServer.remove_interface(ifaces.operationalStatusIface);
    objectServer.remove_interface(ifaces.associationIface);
}

ChassisAssembly::~ChassisAssembly()
{
    removeAssemblyInterfaces(deviceAssembly);
    removeAssemblyInterfaces(boardAssembly);
}

void ChassisAssembly::init()
{
    processNextProperty();
}

void ChassisAssembly::addPropertyTarget(
    gpu::InventoryPropertyId propertyId,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& interface,
    const std::string& propertyName)
{
    properties[propertyId].targets.emplace_back(interface, propertyName);
    properties[propertyId].isPending = true;
}

std::optional<gpu::InventoryPropertyId>
    ChassisAssembly::getNextPendingProperty() const
{
    for (const auto& [propertyId, info] : properties)
    {
        if (info.isPending)
        {
            return propertyId;
        }
    }
    return std::nullopt;
}

void ChassisAssembly::sendRequest(gpu::InventoryPropertyId propertyId)
{
    int rc = gpu::encodeGetInventoryInformationRequest(
        0, static_cast<uint8_t>(propertyId), requestBuffer);
    if (rc != 0)
    {
        lg2::error(
            "ChassisAssembly: Failed to encode property ID {PROP_ID}: rc={RC}",
            "PROP_ID", static_cast<uint8_t>(propertyId), "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, requestBuffer,
        [weak{weak_from_this()}, propertyId](const std::error_code& ec,
                                             std::span<const uint8_t> buffer) {
            std::shared_ptr<ChassisAssembly> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to ChassisAssembly");
                return;
            }
            self->handleResponse(propertyId, ec, buffer);
        });
}

void ChassisAssembly::handleResponse(gpu::InventoryPropertyId propertyId,
                                     const std::error_code& ec,
                                     std::span<const uint8_t> buffer)
{
    auto it = properties.find(propertyId);
    if (it == properties.end())
    {
        processNextProperty();
        return;
    }

    bool success = false;
    if (!ec)
    {
        ocp::accelerator_management::CompletionCode cc{};
        uint16_t reasonCode = 0;
        gpu::InventoryValue info;
        int rc = gpu::decodeGetInventoryInformationResponse(
            buffer, cc, reasonCode, propertyId, info);

        lg2::info(
            "ChassisAssembly: Response for property ID {PROP_ID}, rc={RC}, cc={CC}, reason={REASON}",
            "PROP_ID", static_cast<uint8_t>(propertyId), "RC", rc, "CC",
            static_cast<uint8_t>(cc), "REASON", reasonCode);
        if (rc == 0 &&
            cc == ocp::accelerator_management::CompletionCode::SUCCESS &&
            std::holds_alternative<std::string>(info))
        {
            const std::string& value = std::get<std::string>(info);
            if (!value.empty())
            {
                for (auto& [iface, propName] : it->second.targets)
                {
                    iface->set_property(propName, value);
                }
                success = true;
            }
        }
    }

    if (!success)
    {
        it->second.retryCount++;
        if (it->second.retryCount >= maxRetryAttempts)
        {
            lg2::error(
                "ChassisAssembly: Property ID {PROP_ID} failed after {ATTEMPTS} attempts",
                "PROP_ID", static_cast<uint8_t>(propertyId), "ATTEMPTS",
                maxRetryAttempts);
            it->second.isPending = false;
        }
        else
        {
            retryTimer.expires_after(retryDelay);
            retryTimer.async_wait(
                [weak{weak_from_this()}](const boost::system::error_code& ec) {
                    std::shared_ptr<ChassisAssembly> self = weak.lock();
                    if (!self)
                    {
                        lg2::error("Invalid reference to ChassisAssembly");
                        return;
                    }
                    if (ec)
                    {
                        return;
                    }
                    self->processNextProperty();
                });
            return;
        }
    }
    else
    {
        it->second.isPending = false;
    }

    processNextProperty();
}

void ChassisAssembly::processNextProperty()
{
    std::optional<gpu::InventoryPropertyId> nextProperty =
        getNextPendingProperty();
    if (nextProperty)
    {
        sendRequest(*nextProperty);
    }
}
