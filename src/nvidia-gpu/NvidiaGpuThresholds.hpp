/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <cstdint>
#include <functional>
#include <vector>

using gpuThresholdId = uint8_t;

static constexpr gpuThresholdId gpuTLimitCriticalThresholdId{1};
static constexpr gpuThresholdId gpuTLimitWarnringThresholdId{2};
static constexpr gpuThresholdId gpuTLimitHardshutDownThresholdId{4};

void readThermalParameters(
    uint8_t eid, const std::vector<gpuThresholdId>& ids,
    mctp::MctpRequester& mctpRequester,
    const std::function<void(uint8_t, std::vector<int32_t>)>& callback);
