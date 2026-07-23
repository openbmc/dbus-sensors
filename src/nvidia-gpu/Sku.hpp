/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "Chassis.hpp"
#include "MctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <system_error>

// Issues a single NSM Type 6 Command 1 (QueryGetErotStateParameters) request to
// the GPU's MCTP EID and forwards the decoded AP SKU ID to a Chassis helper.
// The Chassis helper handles publishing Decorator.SKU on the board path.
class Sku : public std::enable_shared_from_this<Sku>
{
  public:
    Sku(mctp::MctpRequester& mctpRequester, uint8_t eid,
        std::shared_ptr<Chassis> chassis);

    void init();

  private:
    void handleResponse(const std::error_code& ec,
                        std::span<const uint8_t> buffer);

    mctp::MctpRequester& mctpRequester;
    uint8_t eid;
    std::shared_ptr<Chassis> chassis;
    std::array<uint8_t, gpu::firmwareGetRotStateRequestSize> requestBuffer{};
};
