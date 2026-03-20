/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <string>

class NvidiaGpuAssembly
{
  public:
    enum class Kind
    {
        Device,
        Board,
    };

    NvidiaGpuAssembly(sdbusplus::asio::object_server& objectServer,
                      const std::string& path, const std::string& chassisPath,
                      Kind kind);
    ~NvidiaGpuAssembly();

    NvidiaGpuAssembly(const NvidiaGpuAssembly&) = delete;
    NvidiaGpuAssembly& operator=(const NvidiaGpuAssembly&) = delete;
    NvidiaGpuAssembly(NvidiaGpuAssembly&&) = delete;
    NvidiaGpuAssembly& operator=(NvidiaGpuAssembly&&) = delete;

    const std::shared_ptr<sdbusplus::asio::dbus_interface>& getAssetIface()
        const
    {
        return assetIface;
    }

  private:
    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<sdbusplus::asio::dbus_interface> panelIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> assetIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> physicalContextIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> embeddedIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> itemIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> operationalStatusIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationIface;
};
