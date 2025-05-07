/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include <GpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <mctp/Requester.hpp>
#include <phosphor-logging/lg2.hpp>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

void readThermalParameter(uint8_t eid, uint8_t id,
                          mctp::MctpRequester& mctpRequester,
                          const std::function<void(uint8_t, int32_t)>& callback)
{
    std::vector<uint8_t> reqMsg(
        sizeof(ocp::accelerator_management::BindingPciVid) +
        sizeof(gpu::ReadThermalParametersRequest));

    auto rc = gpu::encodeReadThermalParametersRequest(0, id, reqMsg);
    if (rc != 0)
    {
        lg2::error("Error reading thermal parameter: encode failed, rc={RC}",
                   "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, reqMsg,
        [callback](int sendRecvMsgResult,
                   std::optional<std::vector<uint8_t>> resp) {
            if (sendRecvMsgResult != 0)
            {
                lg2::error(
                    "Error reading thermal parameter: sending message over MCTP failed, rc={RC}",
                    "RC", sendRecvMsgResult);
                callback(EPROTO, 0);
                return;
            }

            if (!resp.has_value())
            {
                lg2::error("Error reading thermal parameter: empty response");
                callback(EPROTO, 0);
                return;
            }

            const auto& respMsg = resp.value();

            ocp::accelerator_management::CompletionCode cc{};
            uint16_t reasonCode = 0;
            int32_t threshold = 0;

            auto rc = gpu::decodeReadThermalParametersResponse(
                respMsg, cc, reasonCode, threshold);

            if (rc != 0 ||
                cc != ocp::accelerator_management::CompletionCode::SUCCESS)
            {
                lg2::error(
                    "Error reading thermal parameter: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
                    "RC", rc, "CC", cc, "RESC", reasonCode);
                callback(EPROTO, 0);
                return;
            }

            callback(0, threshold);
        });
}

void readThermalParameterCallback(
    uint8_t eid, const std::shared_ptr<std::vector<uint8_t>>& ids,
    mctp::MctpRequester& mctpRequester,
    const std::function<void(uint8_t, std::vector<int32_t>)>& callback,
    size_t index, const std::shared_ptr<std::vector<int32_t>>& thresholds,
    uint8_t rc, int32_t threshold)
{
    if (rc != 0)
    {
        callback(rc, *thresholds);
        return;
    }

    thresholds->push_back(threshold);

    ++index;
    if (index == ids->size())
    {
        callback(rc, *thresholds);
    }
    else
    {
        readThermalParameter(eid, (*ids)[index], mctpRequester,
                             std::bind_front(readThermalParameterCallback, eid,
                                             ids, std::ref(mctpRequester),
                                             callback, index, thresholds));
    }
}

void readThermalParameters(
    uint8_t eid, const std::vector<uint8_t>& ids,
    mctp::MctpRequester& mctpRequester,
    const std::function<void(uint8_t, std::vector<int32_t>)>& callback)
{
    auto thresholds = std::make_shared<std::vector<int32_t>>();
    size_t index = 0;

    readThermalParameter(
        eid, ids[index], mctpRequester,
        std::bind_front(readThermalParameterCallback, eid,
                        std::make_shared<std::vector<uint8_t>>(ids),
                        std::ref(mctpRequester), callback, index, thresholds));
}
