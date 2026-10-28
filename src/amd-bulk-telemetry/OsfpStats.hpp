/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "NetworkAdapter.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace ocp::bulk::osfp_stats
{

inline constexpr uint32_t categoryUidPrefix = 0x0001;

std::optional<Transceiver> decode(std::span<const uint8_t> data);

void update(NetworkAdapter& adapter, size_t instance,
            std::span<const uint8_t> data);

} // namespace ocp::bulk::osfp_stats
