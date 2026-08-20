/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>
#include <string>

constexpr const char* sensorTypeGpu = "NvidiaMctpVdmGpu";
constexpr const char* sensorTypeSma = "NvidiaMctpVdmSma";
constexpr const char* sensorTypeCx = "NvidiaMctpVdmCx";

constexpr uint64_t sensorPollRateMs = 1000;

struct SensorConfigs
{
    std::string name;
    uint64_t pollRate{};
};

// Only a ConnectX device has network ports, so what bounds the probe of them
// is kept apart from the settings every device shares.
struct PcieDeviceConfigs
{
    uint64_t networkPortCount{};
};
