/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>

// Define MCTP EID type
using mctp_eid_t = uint8_t;

namespace mctp
{
/**
 * @brief MCTP requester class
 *
 * This class provides a simple interface for sending and receiving MCTP
 * messages.
 */
class MctpRequester
{};
} // namespace mctp
