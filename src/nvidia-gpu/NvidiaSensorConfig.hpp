/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>
#include <string>

constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";
constexpr const char* deviceType = "NvidiaMctpVdm";

struct SensorConfigs
{
    std::string name;
    uint64_t pollRate{};
    uint64_t nicNetworkPortCount{};
};
