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

namespace ocp::bulk::pcie_state
{

inline constexpr uint32_t categoryUidPrefix = 0x000A;

std::optional<PcieLink> decode(std::span<const uint8_t> data);

void update(NetworkAdapter& adapter, size_t instance,
            std::span<const uint8_t> data);

} // namespace ocp::bulk::pcie_state
