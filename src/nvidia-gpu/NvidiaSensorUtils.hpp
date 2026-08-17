/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "NvidiaGpuMctpVdm.hpp"

#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace nvidia_sensor_utils
{

inline std::optional<std::string> deviceTypeToPhysicalContext(
    gpu::DeviceIdentification deviceType)
{
    switch (deviceType)
    {
        case gpu::DeviceIdentification::DEVICE_GPU:
            return "xyz.openbmc_project.Common.PhysicalContext."
                   "PhysicalContextType.Accelerator";

        case gpu::DeviceIdentification::DEVICE_SMA:
        case gpu::DeviceIdentification::DEVICE_PCIE:
        default:
            // TODO: Define appropriate PhysicalContext for SMA and PCIe devices
            // if needed in the future
            return std::nullopt;
    }
}

inline void registerUpdatedTime(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& iface)
{
    iface->register_property("UpdatedTime", uint64_t{0});
}

// A device that stops responding, or is being reset, leaves the timestamp
// at its last successful update so the value can be seen going stale.
inline void stampUpdatedTime(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& iface)
{
    iface->set_property(
        "UpdatedTime",
        static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count()));
}

} // namespace nvidia_sensor_utils
