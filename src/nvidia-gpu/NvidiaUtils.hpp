/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

constexpr const char* metricPath = "/xyz/openbmc_project/metric/";

// Suffix appended to a GPU inventory path to form its DRAM inventory object,
// e.g. /xyz/openbmc_project/inventory/Nvidia_GPU_0 -> .../Nvidia_GPU_0_DRAM_0
const constexpr char* dramInventorySuffix = "_DRAM_0";

constexpr const char* inventoryPrefix = "/xyz/openbmc_project/inventory/";
