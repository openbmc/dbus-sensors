/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>
#include <span>
#include <system_error>
#include <vector>

/**
 * mock_mctp — test helpers for the mock MctpRequester implementation.
 *
 * The mock .cpp (compiled instead of the real MctpRequester.cpp) calls back
 * synchronously using a configurable response set via these helpers.
 */
namespace mock_mctp
{

void setNextResponse(std::error_code ec, std::vector<uint8_t> response);
void pushResponse(std::error_code ec, std::vector<uint8_t> response);
void clearNextResponse();
void clearHistory();

const std::vector<std::vector<uint8_t>>& getRequestHistory();
size_t getRequestCount();
uint8_t getLastEid();
std::span<const uint8_t> getLastRequest();

} // namespace mock_mctp
