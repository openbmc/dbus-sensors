/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>
#include <string>

constexpr const char* sensorType = "NvidiaMctpVdm";

// Half of a sub-second staleness budget, so a reading still meets it when a
// cycle is delayed or one of its updates fails. Prime, so the poll does not
// stay phase-locked with other periodic work on the BMC.
constexpr uint64_t sensorPollRateMs = 373;

struct SensorConfigs
{
    std::string name;
    uint64_t pollRate{};
    uint64_t nicNetworkPortCount{};
};
