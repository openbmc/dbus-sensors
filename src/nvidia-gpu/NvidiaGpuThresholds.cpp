/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuThresholds.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <vector>

void processReadThermalParameterResponse(
    const std::function<void(uint8_t, int32_t)>& callback,
    const std::span<const uint8_t> respMsg, int sendRecvMsgResult)
{
    if (sendRecvMsgResult != 0)
    {
        lg2::error(
            "Error reading thermal parameter: sending message over MCTP failed, rc={RC}",
            "RC", sendRecvMsgResult);
        callback(EPROTO, 0);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    int32_t threshold = 0;

    auto rc = gpu::decodeReadThermalParametersResponse(respMsg, cc, reasonCode,
                                                       threshold);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error reading thermal parameter: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
            "RC", rc, "CC", cc, "RESC", reasonCode);
        callback(EPROTO, 0);
        return;
    }

    callback(0, threshold);
};

void readThermalParameter(uint8_t eid, uint8_t id,
                          mctp::MctpRequester& mctpRequester,
                          const std::function<void(uint8_t, int32_t)>& callback)
{
    auto reqMsg = std::make_shared<
        std::array<uint8_t, sizeof(gpu::ReadThermalParametersRequest)>>();

    auto respMsg = std::make_shared<
        std::array<uint8_t, sizeof(gpu::ReadThermalParametersResponse)>>();

    auto rc = gpu::encodeReadThermalParametersRequest(0, id, *reqMsg);
    if (rc != 0)
    {
        lg2::error(
            "Error reading thermal parameter for eid {EID} and parameter id {PID} : encode failed. rc={RC}",
            "EID", eid, "PID", id, "RC", rc);
        callback(rc, 0);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, *reqMsg, [reqMsg, callback](int sendRecvMsgResult) {
            std::span<const uint8_t> respMsg;
            processReadThermalParameterResponse(callback, respMsg,
                                                sendRecvMsgResult);
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
        lg2::error(
            "Error reading thermal parameter for eid {EID} and parameter id {PID}. rc={RC}",
            "EID", eid, "PID", (*ids)[index], "RC", rc);
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
