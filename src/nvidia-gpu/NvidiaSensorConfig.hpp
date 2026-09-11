/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sdbusplus/message/native_types.hpp>

#include <cstdint>
#include <string>

constexpr const char* sensorType = "NvidiaMctpVdm";

constexpr uint64_t sensorPollRateMs = 1000;

// What the EntityManager record describing a device says about it: where the
// record lives, and how often the device's sensors are read. A device is
// given one of these rather than the separate arguments it takes today, one
// of which is a path that converts to and from the name beside it without a
// diagnostic.
struct EntityDeviceConfig
{
    sdbusplus::object_path path;
    uint64_t pollRate{};
};
