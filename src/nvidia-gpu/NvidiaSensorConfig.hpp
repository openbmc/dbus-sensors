/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sdbusplus/message/native_types.hpp>

#include <cstdint>
#include <string>

constexpr const char* sensorTypeGpu = "NvidiaMctpVdmGpu";
constexpr const char* sensorTypeSma = "NvidiaMctpVdmSma";
constexpr const char* sensorTypeCx = "NvidiaMctpVdmCx";

constexpr uint64_t sensorPollRateMs = 1000;

// What the EntityManager record describing a device says about it: where the
// record lives, what the device is called on D-Bus, and how often its sensors
// are read.
struct EntityDeviceConfig
{
    sdbusplus::object_path path;
    std::string name;
    uint64_t pollRate{};
};
