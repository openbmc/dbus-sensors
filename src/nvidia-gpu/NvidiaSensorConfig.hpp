/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sdbusplus/message/native_types.hpp>

#include <chrono>
#include <cstdint>
#include <string>

constexpr const char* sensorType = "NvidiaMctpVdm";

constexpr std::chrono::milliseconds sensorPollRate{150};

constexpr std::chrono::seconds roundRobinPollRate{30};

constexpr std::chrono::seconds longRunningPollRate{10};

// An NvidiaMctpVdm record from EntityManager: where it lives, and how often
// the sensors of the device it describes are read. A device is given one of
// these rather than a path beside its name, which converts to and from it
// without a diagnostic.
struct NvidiaMctpVdmConfig
{
    sdbusplus::object_path path;
    uint64_t pollRate{};
};
