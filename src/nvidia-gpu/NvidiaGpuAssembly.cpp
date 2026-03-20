/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuAssembly.hpp"

#include "Utils.hpp"

#include <sdbusplus/asio/object_server.hpp>

#include <string>
#include <vector>

AssemblyInterfaces createDeviceAssembly(
    sdbusplus::asio::object_server& objectServer, const std::string& path,
    const std::string& chassisPath)
{
    AssemblyInterfaces assembly;

    assembly.panelIface = objectServer.add_interface(
        path, "xyz.openbmc_project.Inventory.Item.Panel");
    assembly.panelIface->initialize();

    assembly.assetIface = objectServer.add_interface(
        path, "xyz.openbmc_project.Inventory.Decorator.Asset");
    assembly.assetIface->register_property("Manufacturer",
                                           std::string("NVIDIA"));
    assembly.assetIface->register_property("PartNumber", std::string{});
    assembly.assetIface->register_property("SerialNumber", std::string{});
    assembly.assetIface->register_property("Model", std::string{});
    assembly.assetIface->register_property("BuildDate", std::string{});
    assembly.assetIface->initialize();

    assembly.physicalContextIface = objectServer.add_interface(
        path, "xyz.openbmc_project.Common.PhysicalContext");
    assembly.physicalContextIface->register_property(
        "Type", std::string("xyz.openbmc_project.Common.PhysicalContext."
                            "PhysicalContextType.Accelerator"));
    assembly.physicalContextIface->initialize();

    assembly.embeddedIface = objectServer.add_interface(
        path, "xyz.openbmc_project.Inventory.Connector.Embedded");
    assembly.embeddedIface->initialize();

    assembly.itemIface =
        objectServer.add_interface(path, "xyz.openbmc_project.Inventory.Item");
    assembly.itemIface->register_property("Present", true);
    assembly.itemIface->initialize();

    assembly.operationalStatusIface = objectServer.add_interface(
        path, "xyz.openbmc_project.State.Decorator.OperationalStatus");
    assembly.operationalStatusIface->register_property("Functional", true);
    assembly.operationalStatusIface->initialize();

    std::vector<Association> associations;
    associations.emplace_back("contained_by", "containing", chassisPath);
    assembly.associationIface =
        objectServer.add_interface(path, association::interface);
    assembly.associationIface->register_property("Associations", associations);
    assembly.associationIface->initialize();

    return assembly;
}

AssemblyInterfaces createBoardAssembly(
    sdbusplus::asio::object_server& objectServer, const std::string& path,
    const std::string& chassisPath)
{
    AssemblyInterfaces assembly;

    assembly.panelIface = objectServer.add_interface(
        path, "xyz.openbmc_project.Inventory.Item.Panel");
    assembly.panelIface->initialize();

    assembly.assetIface = objectServer.add_interface(
        path, "xyz.openbmc_project.Inventory.Decorator.Asset");
    assembly.assetIface->register_property("Manufacturer",
                                           std::string("NVIDIA"));
    assembly.assetIface->register_property("PartNumber", std::string{});
    assembly.assetIface->initialize();

    assembly.physicalContextIface = objectServer.add_interface(
        path, "xyz.openbmc_project.Common.PhysicalContext");
    assembly.physicalContextIface->register_property(
        "Type", std::string("xyz.openbmc_project.Common.PhysicalContext."
                            "PhysicalContextType.Accelerator"));
    assembly.physicalContextIface->initialize();

    assembly.embeddedIface = objectServer.add_interface(
        path, "xyz.openbmc_project.Inventory.Connector.Embedded");
    assembly.embeddedIface->initialize();

    assembly.itemIface =
        objectServer.add_interface(path, "xyz.openbmc_project.Inventory.Item");
    assembly.itemIface->register_property("Present", true);
    assembly.itemIface->initialize();

    assembly.operationalStatusIface = objectServer.add_interface(
        path, "xyz.openbmc_project.State.Decorator.OperationalStatus");
    assembly.operationalStatusIface->register_property("Functional", true);
    assembly.operationalStatusIface->initialize();

    std::vector<Association> associations;
    associations.emplace_back("contained_by", "containing", chassisPath);
    assembly.associationIface =
        objectServer.add_interface(path, association::interface);
    assembly.associationIface->register_property("Associations", associations);
    assembly.associationIface->initialize();

    return assembly;
}

void removeAssemblyInterfaces(sdbusplus::asio::object_server& objectServer,
                              AssemblyInterfaces& assembly)
{
    objectServer.remove_interface(assembly.panelIface);
    objectServer.remove_interface(assembly.assetIface);
    objectServer.remove_interface(assembly.physicalContextIface);
    objectServer.remove_interface(assembly.embeddedIface);
    objectServer.remove_interface(assembly.itemIface);
    objectServer.remove_interface(assembly.operationalStatusIface);
    objectServer.remove_interface(assembly.associationIface);
}
