/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaIbPortMetrics.hpp"

#include "NvidiaUtils.hpp"
#include "Utils.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <array>
#include <cstdint>
#include <cstring>
#include <format>
#include <string>
#include <utility>
#include <vector>

static constexpr uint8_t nodeGuidTag = 3;
static constexpr uint8_t portGuidTag = 4;

static std::string formatGuid(uint64_t value)
{
    std::array<uint8_t, sizeof(uint64_t)> guid{};
    std::memcpy(guid.data(), &value, guid.size());
    // Redfish models an InfiniBand GUID as four colon-separated 16-bit
    // groups (XXXX:XXXX:XXXX:XXXX), matching the NetworkDeviceFunction
    // InfiniBand.PermanentNodeGUID / PermanentPortGUID pattern.
    return std::format("{:02X}{:02X}:{:02X}{:02X}:{:02X}{:02X}:{:02X}{:02X}",
                       guid[0], guid[1], guid[2], guid[3], guid[4], guid[5],
                       guid[6], guid[7]);
}

NvidiaIbPortMetrics::NvidiaIbPortMetrics(
    const std::string& name, const std::string& deviceName, uint8_t eid,
    uint16_t portNumber, sdbusplus::asio::object_server& objectServer,
    const std::vector<std::pair<uint8_t, uint64_t>>& addresses)
{
    const sdbusplus::object_path deviceDbusPath = inventoryPrefix / deviceName;

    const sdbusplus::object_path portDbusPath =
        inventoryPrefix / deviceName / name;

    portInterface = objectServer.add_interface(
        portDbusPath, "xyz.openbmc_project.Inventory.Connector.Port");
    portInterface->register_property(
        "PortProtocol",
        std::string("xyz.openbmc_project.Inventory.Connector.Port."
                    "PortProtocol.InfiniBand"));
    if (!portInterface->initialize())
    {
        lg2::error("Error initializing IB Port interface, eid={EID}, port={PN}",
                   "EID", eid, "PN", portNumber);
    }

    std::vector<Association> associations;
    associations.emplace_back("connected_to", "connecting", deviceDbusPath);
    associationInterface =
        objectServer.add_interface(portDbusPath, association::interface);
    associationInterface->register_property("Associations", associations);
    if (!associationInterface->initialize())
    {
        lg2::error(
            "Error initializing IB Port Association interface, eid={EID}, port={PN}",
            "EID", eid, "PN", portNumber);
    }

    std::string permanentNodeGuid;
    std::string permanentPortGuid;
    for (const auto& [tag, value] : addresses)
    {
        if (tag == nodeGuidTag)
        {
            permanentNodeGuid = formatGuid(value);
        }
        else if (tag == portGuidTag)
        {
            permanentPortGuid = formatGuid(value);
        }
    }

    const sdbusplus::object_path ndfDbusPath =
        inventoryPrefix / deviceName / "NetworkDeviceFunctions" / name;

    networkDeviceFunctionInterface = objectServer.add_interface(
        ndfDbusPath, "xyz.openbmc_project.Inventory.Item.NetworkInterface");
    networkDeviceFunctionInterface->register_property("PermanentNodeGUID",
                                                      permanentNodeGuid);
    networkDeviceFunctionInterface->register_property("PermanentPortGUID",
                                                      permanentPortGuid);
    if (!networkDeviceFunctionInterface->initialize())
    {
        lg2::error(
            "Error initializing IB NetworkDeviceFunction interface, eid={EID}, port={PN}",
            "EID", eid, "PN", portNumber);
    }

    std::vector<Association> ndfAssociations;
    ndfAssociations.emplace_back("exposed_by", "exposing", deviceDbusPath);
    ndfAssociations.emplace_back("assigned_to", "assigning", portDbusPath);
    networkDeviceFunctionAssociationInterface =
        objectServer.add_interface(ndfDbusPath, association::interface);
    networkDeviceFunctionAssociationInterface->register_property(
        "Associations", ndfAssociations);
    if (!networkDeviceFunctionAssociationInterface->initialize())
    {
        lg2::error(
            "Error initializing IB NetworkDeviceFunction Association interface, eid={EID}, port={PN}",
            "EID", eid, "PN", portNumber);
    }
}
