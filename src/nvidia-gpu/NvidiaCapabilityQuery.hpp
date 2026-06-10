/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <system_error>
#include <utility>
#include <vector>

// Queries a device's supported message types, then its supported command codes
// for each. Any failure hands back caps with queried=false.
class CapabilityQuery : public std::enable_shared_from_this<CapabilityQuery>
{
  public:
    CapabilityQuery(
        uint8_t eid, mctp::MctpRequester& mctpRequester,
        std::function<void(const gpu::DeviceCapabilities&)> onComplete) :
        eid(eid), mctpRequester(mctpRequester),
        onComplete(std::move(onComplete))
    {}

    void start()
    {
        request.assign(gpu::getSupportedMessageTypesRequestSize, 0);
        if (gpu::encodeGetSupportedMessageTypesRequest(0, request) != 0)
        {
            finish();
            return;
        }
        mctpRequester.sendRecvMsg(
            eid, request,
            [self = shared_from_this()](const std::error_code& ec,
                                        std::span<const uint8_t> resp) {
                self->onMessageTypes(ec, resp);
            });
    }

  private:
    void onMessageTypes(const std::error_code& ec,
                        std::span<const uint8_t> resp)
    {
        ocp::accelerator_management::CompletionCode cc{};
        uint16_t reasonCode = 0;
        std::array<uint8_t, gpu::supportedListBitfieldSize> bitmap{};

        if (ec ||
            gpu::decodeGetSupportedMessageTypesResponse(resp, cc, reasonCode,
                                                        bitmap) != 0 ||
            cc != ocp::accelerator_management::CompletionCode::SUCCESS)
        {
            lg2::error(
                "EID {EID}: GetSupportedMessageTypes failed; creating all sensors",
                "EID", eid);
            finish();
            return;
        }

        for (auto type :
             {gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY,
              gpu::MessageType::NETWORK_PORT, gpu::MessageType::PCIE_LINK,
              gpu::MessageType::PLATFORM_ENVIRONMENTAL})
        {
            auto bit = static_cast<uint8_t>(type);
            if ((bitmap[bit / 8] & (1U << (bit % 8))) != 0)
            {
                typesToQuery.push_back(type);
            }
        }

        queryNextType();
    }

    void queryNextType()
    {
        if (index >= typesToQuery.size())
        {
            caps.queried = true;
            finish();
            return;
        }

        request.assign(gpu::getSupportedCommandCodesRequestSize, 0);
        if (gpu::encodeGetSupportedCommandCodesRequest(
                0, static_cast<uint8_t>(typesToQuery[index]), request) != 0)
        {
            finish();
            return;
        }
        mctpRequester.sendRecvMsg(
            eid, request,
            [self = shared_from_this()](const std::error_code& ec,
                                        std::span<const uint8_t> resp) {
                self->onCommandCodes(ec, resp);
            });
    }

    void onCommandCodes(const std::error_code& ec,
                        std::span<const uint8_t> resp)
    {
        gpu::MessageType type = typesToQuery[index];
        ocp::accelerator_management::CompletionCode cc{};
        uint16_t reasonCode = 0;
        std::array<uint8_t, gpu::supportedListBitfieldSize> bitmap{};

        if (ec)
        {
            lg2::error("EID {EID}: GetSupportedCommandCodes for type {TYPE} "
                       "failed: MCTP transport error: {ERROR}",
                       "EID", eid, "TYPE", static_cast<int>(type), "ERROR",
                       ec.message());
            abandon();
            return;
        }

        if (int rc = gpu::decodeGetSupportedCommandCodesResponse(
                resp, cc, reasonCode, bitmap);
            rc != 0)
        {
            lg2::error("EID {EID}: GetSupportedCommandCodes for type {TYPE} "
                       "failed: decode error rc={RC}",
                       "EID", eid, "TYPE", static_cast<int>(type), "RC", rc);
            abandon();
            return;
        }

        if (cc != ocp::accelerator_management::CompletionCode::SUCCESS)
        {
            lg2::error("EID {EID}: GetSupportedCommandCodes for type {TYPE} "
                       "failed: cc={CC} reasonCode={RSN}",
                       "EID", eid, "TYPE", static_cast<int>(type), "CC",
                       static_cast<int>(cc), "RSN", reasonCode);
            abandon();
            return;
        }

        caps.commands[type] = bitmap;

        ++index;
        queryNextType();
    }

    // A partial result would gate off every command of the types that were
    // not answered, so drop it and fall back to querying nothing.
    void abandon()
    {
        caps.queried = false;
        caps.commands.clear();
        finish();
    }

    void finish()
    {
        onComplete(caps);
    }

    uint8_t eid;
    mctp::MctpRequester& mctpRequester;
    std::function<void(const gpu::DeviceCapabilities&)> onComplete;
    gpu::DeviceCapabilities caps;
    std::vector<gpu::MessageType> typesToQuery;
    size_t index{0};
    std::vector<uint8_t> request;
};
