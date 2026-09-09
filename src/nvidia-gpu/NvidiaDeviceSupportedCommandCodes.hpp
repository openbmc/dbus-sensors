/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <span>
#include <system_error>
#include <vector>

namespace gpu
{

class DeviceSupportedCommandCodes :
    public std::enable_shared_from_this<DeviceSupportedCommandCodes>
{
  public:
    DeviceSupportedCommandCodes(uint8_t eid,
                                mctp::MctpRequester& mctpRequester) :
        eid(eid), mctpRequester(mctpRequester)
    {}

    void refresh(std::function<void()> onComplete);

    bool supports(DeviceCapabilityDiscoveryCommands command) const;
    bool supports(NetworkPortCommands command) const;
    bool supports(PcieLinkCommands command) const;
    bool supports(PlatformEnvironmentalCommands command) const;

  private:
    void onMessageTypes(const std::error_code& ec,
                        std::span<const uint8_t> response);
    void queryNextType();
    void onCommandCodes(const std::error_code& ec,
                        std::span<const uint8_t> response);
    void abandon();
    void finish();
    bool has(MessageType type, uint8_t command) const;

    uint8_t eid;
    mctp::MctpRequester& mctpRequester;
    std::function<void()> onRefreshComplete;
    bool inFlight{false};
    bool refreshPending{false};
    bool queried{false};
    std::map<MessageType, std::array<uint8_t, supportedListBitfieldSize>>
        commands;
    std::map<MessageType, std::array<uint8_t, supportedListBitfieldSize>>
        pendingCommands;
    std::vector<MessageType> typesToQuery;
    size_t index{0};

    std::array<uint8_t, getSupportedMessageTypesRequestSize>
        getSupportedMessageTypesRequestBuffer{};
    std::array<uint8_t, getSupportedCommandCodesRequestSize>
        getSupportedCommandCodesRequestBuffer{};
};

} // namespace gpu
