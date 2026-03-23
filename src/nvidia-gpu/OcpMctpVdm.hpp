/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MessagePackUnpackUtils.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>

namespace ocp
{
namespace accelerator_management
{

constexpr uint8_t messageType = 0x7E;

constexpr uint8_t ocpType = 8;
constexpr uint8_t ocpVersion = 9;
constexpr uint8_t ocpTypeBitOffset = 4;
constexpr uint8_t ocpTypeBitMask = 0b11110000;
constexpr uint8_t ocpVersionBitMask = 0b00001111;
constexpr uint8_t instanceIdBitMask = 0b00011111;
constexpr uint8_t instanceIdReservedBitMask = 0b00100000;
constexpr uint8_t datagramBitMask = 0b01000000;
constexpr uint8_t requestBitMask = 0b10000000;

constexpr uint8_t instanceMin = 0;
constexpr uint8_t instanceMax = 31;

enum class CompletionCode : uint8_t
{
    SUCCESS = 0x00,
    ERROR = 0x01,
    ERR_INVALID_DATA = 0x02,
    ERR_INVALID_DATA_LENGTH = 0x03,
    ERR_NOT_READY = 0x04,
    ERR_UNSUPPORTED_COMMAND_CODE = 0x05,
    ERR_UNSUPPORTED_MSG_TYPE = 0x06,
    ERR_BUS_ACCESS = 0x7f,
    ERR_NULL = 0x80,
};

enum class ReasonCode : uint16_t
{
    REASON_NONE = 0x00,
};

enum class MessageType : uint8_t
{
    RESPONSE = 0, //!< OCP MCTP VDM response message
    REQUEST = 2,  //!< OCP MCTP VDM request message
};

constexpr size_t messageHeaderSize =
    sizeof(uint16_t) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint8_t);

constexpr size_t instanceIdOffset = sizeof(uint16_t);

constexpr size_t commonRequestSize =
    messageHeaderSize + sizeof(uint8_t) + sizeof(uint8_t);

constexpr size_t commonResponseSize =
    messageHeaderSize + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint16_t) +
    sizeof(uint16_t);

int packHeader(PackBuffer& buffer, uint16_t pciVendorId,
               MessageType ocpAcceleratorManagementMsgType, uint8_t instanceId,
               uint8_t msgType);

int unpackHeader(UnpackBuffer& buffer, uint16_t pciVendorId,
                 MessageType& ocpAcceleratorManagementMsgType,
                 uint8_t& instanceId, uint8_t& msgType);

int unpackReasonCodeAndCC(UnpackBuffer& buffer, CompletionCode& cc,
                          uint16_t& reasonCode);

int unpackAggregateResponse(
    UnpackBuffer& buffer,
    std::move_only_function<int(const uint8_t tag, const uint8_t length,
                                UnpackBuffer& buffer)>
        handler);

} // namespace accelerator_management
} // namespace ocp
