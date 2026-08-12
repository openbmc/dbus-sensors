/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sdbusplus/exception.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cerrno>

constexpr const char* metricPath = "/xyz/openbmc_project/metric/";
constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";

// DRAM inventory name segment, joined to a GPU name with a '_' separator to
// form its DRAM inventory object, e.g. Nvidia_GPU_0 -> Nvidia_GPU_0_DRAM_0
constexpr const char* dramInventorySuffix = "DRAM_0";

constexpr const char* nvidiaManufacturer = "NVIDIA";

inline const sdbusplus::object_path inventoryPrefix{
    "/xyz/openbmc_project/inventory"};

// Lightweight D-Bus error types for the Control property-set handlers, so the
// app does not depend on the phosphor-dbus-interfaces C++ bindings. Same
// approach as SetSensorError in sensor.hpp.
struct Unavailable : sdbusplus::exception_t
{
    const char* name() const noexcept override
    {
        return "xyz.openbmc_project.Common.Error.Unavailable";
    }
    const char* description() const noexcept override
    {
        return "The device is currently unavailable.";
    }
    int get_errno() const noexcept override
    {
        return EAGAIN;
    }
};

struct InvalidArgument : sdbusplus::exception_t
{
    const char* name() const noexcept override
    {
        return "xyz.openbmc_project.Common.Error.InvalidArgument";
    }
    const char* description() const noexcept override
    {
        return "Invalid argument.";
    }
    int get_errno() const noexcept override
    {
        return EINVAL;
    }
};
