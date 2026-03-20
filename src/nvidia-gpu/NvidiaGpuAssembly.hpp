/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <string>

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

AssemblyInterfaces createDeviceAssembly(
    sdbusplus::asio::object_server& objectServer, const std::string& path,
    const std::string& chassisPath);

AssemblyInterfaces createBoardAssembly(
    sdbusplus::asio::object_server& objectServer, const std::string& path,
    const std::string& chassisPath);

void removeAssemblyInterfaces(sdbusplus::asio::object_server& objectServer,
                              AssemblyInterfaces& assembly);
