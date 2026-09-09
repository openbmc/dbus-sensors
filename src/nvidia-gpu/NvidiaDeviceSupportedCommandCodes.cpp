/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaDeviceSupportedCommandCodes.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>

#include <array>
#include <cstdint>
#include <functional>
#include <span>
#include <system_error>
#include <utility>

namespace gpu
{

void DeviceSupportedCommandCodes::refresh(std::function<void()> onComplete)
{
    if (inFlight)
    {
        refreshPending = true;
        if (onComplete)
        {
            lg2::error("EID {EID}: a refresh is already in flight, dropping "
                       "the completion handler of the new one",
                       "EID", eid);
        }
        return;
    }

    inFlight = true;
    onRefreshComplete = std::move(onComplete);
    typesToQuery.clear();
    pendingCommands.clear();

    if (encodeGetSupportedMessageTypesRequest(
            0, getSupportedMessageTypesRequestBuffer) != 0)
    {
        lg2::error("EID {EID}: failed to encode GetSupportedMessageTypes",
                   "EID", eid);
        abandon();
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, getSupportedMessageTypesRequestBuffer,
        [weak{weak_from_this()}, eid{eid}](const std::error_code& ec,
                                           std::span<const uint8_t> response) {
            const auto self = weak.lock();
            if (!self)
            {
                lg2::error(
                    "EID {EID}: DeviceSupportedCommandCodes expired in GetSupportedMessageTypes callback",
                    "EID", eid);
                return;
            }
            self->onMessageTypes(ec, response);
        });
}

void DeviceSupportedCommandCodes::onMessageTypes(
    const std::error_code& ec, std::span<const uint8_t> response)
{
    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    std::array<uint8_t, supportedListBitfieldSize> bitmap{};

    if (ec ||
        decodeGetSupportedMessageTypesResponse(response, cc, reasonCode,
                                               bitmap) != 0 ||
        cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error("EID {EID}: GetSupportedMessageTypes failed", "EID", eid);
        abandon();
        return;
    }

    index = 0;
    for (const auto type :
         {MessageType::DEVICE_CAPABILITY_DISCOVERY, MessageType::NETWORK_PORT,
          MessageType::PCIE_LINK, MessageType::PLATFORM_ENVIRONMENTAL})
    {
        const auto bit = static_cast<uint8_t>(type);
        if ((bitmap[bit / 8U] & (1U << (bit % 8U))) != 0)
        {
            typesToQuery.push_back(type);
        }
    }

    queryNextType();
}

void DeviceSupportedCommandCodes::queryNextType()
{
    if (index >= typesToQuery.size())
    {
        commands = std::move(pendingCommands);
        pendingCommands.clear();
        queried = true;
        finish();
        return;
    }

    if (encodeGetSupportedCommandCodesRequest(
            0, static_cast<uint8_t>(typesToQuery[index]),
            getSupportedCommandCodesRequestBuffer) != 0)
    {
        lg2::error("EID {EID}: failed to encode GetSupportedCommandCodes for "
                   "type {TYPE}",
                   "EID", eid, "TYPE", static_cast<int>(typesToQuery[index]));
        abandon();
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, getSupportedCommandCodesRequestBuffer,
        [weak{weak_from_this()}, eid{eid}](const std::error_code& ec,
                                           std::span<const uint8_t> response) {
            const auto self = weak.lock();
            if (!self)
            {
                lg2::error(
                    "EID {EID}: DeviceSupportedCommandCodes expired in GetSupportedCommandCodes callback",
                    "EID", eid);
                return;
            }
            self->onCommandCodes(ec, response);
        });
}

void DeviceSupportedCommandCodes::onCommandCodes(
    const std::error_code& ec, std::span<const uint8_t> response)
{
    const MessageType type = typesToQuery[index];
    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    std::array<uint8_t, supportedListBitfieldSize> bitmap{};

    if (ec)
    {
        lg2::error("EID {EID}: GetSupportedCommandCodes for type {TYPE} "
                   "failed: MCTP transport error: {ERROR}",
                   "EID", eid, "TYPE", static_cast<int>(type), "ERROR",
                   ec.message());
        abandon();
        return;
    }

    if (const int rc = decodeGetSupportedCommandCodesResponse(
            response, cc, reasonCode, bitmap);
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

    pendingCommands[type] = bitmap;

    ++index;
    queryNextType();
}

// A partial set would gate off every command of the types that were not
// answered, so the answered types are only committed once every type has been
// read.
void DeviceSupportedCommandCodes::abandon()
{
    pendingCommands.clear();
    finish();
}

void DeviceSupportedCommandCodes::finish()
{
    inFlight = false;

    if (onRefreshComplete)
    {
        const auto callback = std::move(onRefreshComplete);
        onRefreshComplete = nullptr;
        callback();
    }

    if (refreshPending)
    {
        refreshPending = false;
        refresh(nullptr);
    }
}

bool DeviceSupportedCommandCodes::has(MessageType type, uint8_t command) const
{
    if (!queried)
    {
        return true;
    }
    const auto it = commands.find(type);
    return it != commands.end() &&
           (it->second[command / 8U] & (1U << (command % 8U))) != 0;
}

bool DeviceSupportedCommandCodes::supports(
    DeviceCapabilityDiscoveryCommands command) const
{
    return has(MessageType::DEVICE_CAPABILITY_DISCOVERY,
               static_cast<uint8_t>(command));
}

bool DeviceSupportedCommandCodes::supports(NetworkPortCommands command) const
{
    return has(MessageType::NETWORK_PORT, static_cast<uint8_t>(command));
}

bool DeviceSupportedCommandCodes::supports(PcieLinkCommands command) const
{
    return has(MessageType::PCIE_LINK, static_cast<uint8_t>(command));
}

bool DeviceSupportedCommandCodes::supports(
    PlatformEnvironmentalCommands command) const
{
    return has(MessageType::PLATFORM_ENVIRONMENTAL,
               static_cast<uint8_t>(command));
}

} // namespace gpu
