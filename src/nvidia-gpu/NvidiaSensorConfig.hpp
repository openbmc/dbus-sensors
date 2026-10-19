/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sdbusplus/message/native_types.hpp>

#include <chrono>
#include <cstdint>
#include <string>

constexpr const char* sensorTypeGpu = "NvidiaMctpVdmGpu";
constexpr const char* sensorTypeSma = "NvidiaMctpVdmSma";
constexpr const char* sensorTypeCx = "NvidiaMctpVdmCx";

constexpr std::chrono::milliseconds sensorPollRate{150};

constexpr std::chrono::seconds roundRobinPollRate{30};

constexpr std::chrono::seconds longRunningPollRate{10};

// An NvidiaMctpVdmGpu record from EntityManager: where it lives, what the GPU
// it describes is called on D-Bus, and how often its sensors are read.
struct GpuDeviceConfig
{
    sdbusplus::object_path path;
    std::string name;
    uint64_t pollRate{};
};

// An NvidiaMctpVdmSma record from EntityManager, with the same settings for
// an SMA.
struct SmaDeviceConfig
{
    sdbusplus::object_path path;
    std::string name;
    uint64_t pollRate{};
};

// An NvidiaMctpVdmCx record from EntityManager, which also says how many
// network ports the ConnectX has.
struct CxDeviceConfig
{
    sdbusplus::object_path path;
    std::string name;
    uint64_t pollRate{};
    uint64_t portCount{};
};
