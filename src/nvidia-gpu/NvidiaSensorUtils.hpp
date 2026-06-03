/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "NvidiaGpuMctpVdm.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace nvidia_sensor_utils
{

namespace temperature_sensor_id
{
inline constexpr uint8_t device{0};
inline constexpr uint8_t dram{1};
inline constexpr uint8_t tLimit{2};
inline constexpr uint8_t sma{17};
} // namespace temperature_sensor_id

namespace power_energy_sensor_id
{
inline constexpr uint8_t device{0};
} // namespace power_energy_sensor_id

namespace voltage_sensor_id
{
inline constexpr uint8_t gpu{0};
} // namespace voltage_sensor_id

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

} // namespace nvidia_sensor_utils
