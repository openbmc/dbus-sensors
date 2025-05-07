/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "mctp/Requester.hpp"

#include <cstdint>
#include <functional>
#include <vector>

using gpuThresholdId = uint8_t;

constexpr gpuThresholdId gpuTLimitCriticalThresholdId{1};
constexpr gpuThresholdId gpuTLimitWarnringThresholdId{2};
constexpr gpuThresholdId gpuTLimitHardshutDownThresholdId{4};

/** @brief Read multiple thermal parameters
 *
 *  @param[in] eid - Endpoint ID
 *  @param[in] ids - Shared pointer to vector of sensor IDs to read
 *  @param[in] mctpRequester - Reference to MCTP requester
 *  @param[in] callback - Callback function to process results
 *              Takes sensor ID and vector of threshold values
 */
void readThermalParameters(
    uint8_t eid, const std::vector<gpuThresholdId>& ids,
    mctp::MctpRequester& mctpRequester,
    const std::function<void(uint8_t, std::vector<int32_t>)>& callback);
