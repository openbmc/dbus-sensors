/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <cstdint>
#include <functional>
#include <vector>

using gpuThresholdId = uint8_t;

constexpr gpuThresholdId gpuTLimitCriticalThresholdId{1};
constexpr gpuThresholdId gpuTLimitWarnringThresholdId{2};
constexpr gpuThresholdId gpuTLimitHardshutDownThresholdId{4};

void readThermalParameters(
    uint8_t eid, const std::vector<gpuThresholdId>& ids,
    mctp::MctpRequester& mctpRequester,
    const std::function<void(uint8_t, std::vector<int32_t>)>& callback);
