/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuAssembly.hpp"

#include "NvidiaUtils.hpp"
#include "Utils.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <string>
#include <vector>

NvidiaGpuAssembly::NvidiaGpuAssembly(
    sdbusplus::asio::object_server& objectServer, const std::string& path,
    const std::string& chassisPath, Kind kind) : objectServer(objectServer)
{
    panelIface = objectServer.add_interface(
        path, "xyz.openbmc_project.Inventory.Item.Panel");
    if (!panelIface->initialize())
    {
        lg2::error("Failed to initialize panel interface for {PATH}", "PATH",
                   path);
    }

    assetIface = objectServer.add_interface(
        path, "xyz.openbmc_project.Inventory.Decorator.Asset");
    assetIface->register_property("Manufacturer",
                                  std::string(nvidiaManufacturer));
    assetIface->register_property("PartNumber", std::string{});
    if (kind == Kind::Device)
    {
        assetIface->register_property("SerialNumber", std::string{});
        assetIface->register_property("Model", std::string{});
        assetIface->register_property("BuildDate", std::string{});
    }
    if (!assetIface->initialize())
    {
        lg2::error("Failed to initialize asset interface for {PATH}", "PATH",
                   path);
    }

    physicalContextIface = objectServer.add_interface(
        path, "xyz.openbmc_project.Common.PhysicalContext");
    physicalContextIface->register_property(
        "Type", std::string("xyz.openbmc_project.Common.PhysicalContext."
                            "PhysicalContextType.Accelerator"));
    if (!physicalContextIface->initialize())
    {
        lg2::error("Failed to initialize physical context interface for {PATH}",
                   "PATH", path);
    }

    embeddedIface = objectServer.add_interface(
        path, "xyz.openbmc_project.Inventory.Connector.Embedded");
    if (!embeddedIface->initialize())
    {
        lg2::error("Failed to initialize embedded interface for {PATH}", "PATH",
                   path);
    }

    itemIface =
        objectServer.add_interface(path, "xyz.openbmc_project.Inventory.Item");
    itemIface->register_property("Present", true);
    if (!itemIface->initialize())
    {
        lg2::error("Failed to initialize item interface for {PATH}", "PATH",
                   path);
    }

    operationalStatusIface = objectServer.add_interface(
        path, "xyz.openbmc_project.State.Decorator.OperationalStatus");
    operationalStatusIface->register_property("Functional", true);
    if (!operationalStatusIface->initialize())
    {
        lg2::error(
            "Failed to initialize operational status interface for {PATH}",
            "PATH", path);
    }

    std::vector<Association> associations;
    associations.emplace_back("contained_by", "containing", chassisPath);
    associationIface = objectServer.add_interface(path, association::interface);
    associationIface->register_property("Associations", associations);
    if (!associationIface->initialize())
    {
        lg2::error("Failed to initialize association interface for {PATH}",
                   "PATH", path);
    }
}

NvidiaGpuAssembly::~NvidiaGpuAssembly()
{
    objectServer.remove_interface(panelIface);
    objectServer.remove_interface(assetIface);
    objectServer.remove_interface(physicalContextIface);
    objectServer.remove_interface(embeddedIface);
    objectServer.remove_interface(itemIface);
    objectServer.remove_interface(operationalStatusIface);
    objectServer.remove_interface(associationIface);
}
