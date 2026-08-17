/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <string>

constexpr const char* sensorType = "NvidiaMctpVdm";

// Power and thermal readings close control loops and carry a sub-second
// staleness budget, so they are polled on an interval of their own.
constexpr uint64_t sensorPollRateMs = 150;

// The remaining readings change over the life of a workload rather than
// within one, so a bound on how stale they may get is enough. Holding them
// off the priority interval leaves the transport to the readings that need
// it, since a device answers one request at a time.
constexpr std::chrono::seconds roundRobinPollRate{30};

// A long running command is acknowledged first and answered later, and can
// take up to 2s when it times out, so it is kept off both intervals above.
// Four of them fit in this period even if every one times out.
constexpr std::chrono::seconds longRunningPollRate{10};

struct SensorConfigs
{
    std::string name;
    uint64_t pollRate{};
    uint64_t nicNetworkPortCount{};
};
