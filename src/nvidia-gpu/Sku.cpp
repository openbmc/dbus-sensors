/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Sku.hpp"

#include "Chassis.hpp"
#include "MctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"

#include <phosphor-logging/lg2.hpp>

#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>

Sku::Sku(mctp::MctpRequester& mctpRequester, uint8_t eid,
         std::shared_ptr<Chassis> chassis) :
    mctpRequester(mctpRequester), eid(eid), chassis(std::move(chassis))
{}

void Sku::init()
{
    int rc = gpu::encodeFirmwareGetRotStateRequest(
        0, gpu::componentClassificationAp, gpu::componentIdentifierGpuAp, 0,
        requestBuffer);
    if (rc != 0)
    {
        lg2::error("Failed to encode SKU request for EID {EID}: rc={RC}", "EID",
                   eid, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, requestBuffer,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<Sku> self = weak.lock();
            if (!self)
            {
                return;
            }
            self->handleResponse(ec, buffer);
        });
}

void Sku::handleResponse(const std::error_code& ec,
                         std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::debug("SKU request transport error for EID {EID}: {ERR}", "EID",
                   eid, "ERR", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t apSkuId = 0;
    int rc = gpu::decodeFirmwareGetRotStateApSkuIdResponse(
        buffer, cc, reasonCode, apSkuId);
    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::debug(
            "SKU response decode failed for EID {EID}: rc={RC} cc={CC} reason={REASON}",
            "EID", eid, "RC", rc, "CC", static_cast<uint8_t>(cc), "REASON",
            reasonCode);
        return;
    }

    // AP_SKU_ID is optional in the RoT state response; when the firmware does
    // not report it the value decodes to 0 and is published as 0x00000000,
    // matching the nsmd AP SKU ID behavior.
    std::string skuStr = std::format("0x{:08X}", apSkuId);
    if (chassis)
    {
        chassis->onSku(skuStr);
    }
}
