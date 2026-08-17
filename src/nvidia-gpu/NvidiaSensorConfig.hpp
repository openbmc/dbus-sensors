/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <string>

constexpr const char* sensorType = "NvidiaMctpVdm";

constexpr uint64_t sensorPollRateMs = 150;

constexpr std::chrono::seconds roundRobinPollRate{30};

constexpr std::chrono::seconds longRunningPollRate{10};

struct SensorConfigs
{
    std::string name;
    uint64_t pollRate{};
    uint64_t nicNetworkPortCount{};
};
