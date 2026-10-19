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

struct GpuDeviceConfig
{
    sdbusplus::object_path path;
    std::string name;
    uint64_t pollRate{};
};

struct SmaDeviceConfig
{
    sdbusplus::object_path path;
    std::string name;
    uint64_t pollRate{};
};

struct CxDeviceConfig
{
    sdbusplus::object_path path;
    std::string name;
    uint64_t pollRate{};
    uint64_t portCount{};
};
