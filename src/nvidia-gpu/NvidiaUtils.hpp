/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sdbusplus/message/native_types.hpp>

#include <string>

constexpr const char* metricPath = "/xyz/openbmc_project/metric/";
constexpr const char* metricInterface = "xyz.openbmc_project.Metric.Value";
constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";

// Units for the Value property of xyz.openbmc_project.Metric.Value.
inline const std::string metricUnitBytes =
    "xyz.openbmc_project.Metric.Value.Unit.Bytes";
inline const std::string metricUnitCount =
    "xyz.openbmc_project.Metric.Value.Unit.Count";

// DRAM inventory name segment, joined to a GPU name with a '_' separator to
// form its DRAM inventory object, e.g. Nvidia_GPU_0 -> Nvidia_GPU_0_DRAM_0
constexpr const char* dramInventorySuffix = "DRAM_0";

constexpr const char* nvidiaManufacturer = "NVIDIA";

inline const sdbusplus::object_path inventoryPrefix{
    "/xyz/openbmc_project/inventory"};
