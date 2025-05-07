/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include <GpuMctpVdm.hpp>
#include <MctpRequester.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

void readThermalParameter(uint8_t eid, uint8_t id,
                          mctp::MctpRequester& mctpRequester,
                          const std::function<void(uint8_t, int32_t)>& callback)
{
    std::vector<uint8_t> reqMsg(
        sizeof(ocp::accelerator_management::BindingPciVid) +
        sizeof(gpu::ReadThermalParametersRequest));

    auto* msg = new (reqMsg.data()) ocp::accelerator_management::Message;

    auto rc = gpu::encodeReadThermalParametersRequest(0, id, *msg);
    if (rc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error("encodeReadThermalParametersRequest failed, rc={RC}", "RC",
                   static_cast<int>(rc));

        callback(static_cast<uint8_t>(
                     ocp::accelerator_management::CompletionCode::ERROR),
                 0);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, reqMsg,
        [callback](int sendRecvMsgResult, std::vector<uint8_t> respMsg) {
            if (sendRecvMsgResult != 0)
            {
                lg2::error("MctpRequester::sendRecvMsg() failed, rc={RC}", "RC",
                           sendRecvMsgResult);

                callback(
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0);
                return;
            }

            if (respMsg.empty())
            {
                lg2::error("MctpRequester::sendRecvMsg() failed, respMsgLen=0");

                callback(
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0);
                return;
            }

            uint8_t cc = 0;
            uint16_t reasonCode = 0;
            int32_t threshold = 0;

            auto rc = gpu::decodeReadThermalParametersResponse(
                *new (respMsg.data()) ocp::accelerator_management::Message,
                respMsg.size(), cc, reasonCode, threshold);

            if (rc != ocp::accelerator_management::CompletionCode::SUCCESS ||
                cc != static_cast<uint8_t>(
                          ocp::accelerator_management::CompletionCode::SUCCESS))
            {
                lg2::error(
                    "decodeReadThermalParametersResponse() failed, rc={RC} cc={CC} reasonCode={RESC}",
                    "RC", static_cast<int>(rc), "CC", cc, "RESC", reasonCode);

                callback(
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0);
                return;
            }

            callback(static_cast<uint8_t>(
                         ocp::accelerator_management::CompletionCode::SUCCESS),
                     0);
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
