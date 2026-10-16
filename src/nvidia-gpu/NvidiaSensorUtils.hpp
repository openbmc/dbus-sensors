/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "NvidiaGpuMctpVdm.hpp"

#include <algorithm>
#include <array>
#include <format>
#include <optional>
#include <string>
#include <string_view>

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

inline std::optional<std::string> nameToPhysicalContext(std::string_view name)
{
    constexpr std::array<std::string_view, 5> names{
        "Board", "CPU", "DCBus", "GPUSubsystem", "VoltageRegulator"};
    if (std::ranges::find(names, name) == names.end())
    {
        return std::nullopt;
    }
    return std::format(
        "xyz.openbmc_project.Common.PhysicalContext.PhysicalContextType.{}",
        name);
}

} // namespace nvidia_sensor_utils
