/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "NetworkAdapter.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace ocp::bulk::xrmac_stats
{

inline constexpr uint32_t categoryUidPrefix = 0x0006;

std::vector<PortCounter> decode(std::span<const uint8_t> data);

void update(NetworkAdapter& adapter, size_t instance,
            std::span<const uint8_t> data);

} // namespace ocp::bulk::xrmac_stats
