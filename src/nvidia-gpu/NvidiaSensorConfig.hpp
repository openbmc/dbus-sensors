/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>
#include <string>

// The configuration type that does not name a device kind, kept for
// configurations that have not moved to the per kind ones below.
constexpr const char* sensorType = "NvidiaMctpVdm";
constexpr const char* sensorTypeGpu = "NvidiaMctpVdmGpu";
constexpr const char* sensorTypeSma = "NvidiaMctpVdmSma";
constexpr const char* sensorTypeCx = "NvidiaMctpVdmCx";

struct SensorConfigs
{
    std::string name;
    uint64_t pollRate{};
    uint64_t nicNetworkPortCount{};
};
