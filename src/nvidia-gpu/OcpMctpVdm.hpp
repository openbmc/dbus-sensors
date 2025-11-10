/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>
#include <functional>
#include <span>

class PackBuffer;
class UnpackBuffer;

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

struct BindingPciVid
{
    uint16_t pci_vendor_id;                      //!< PCI defined vendor ID
    uint8_t instance_id;                         //!< Instance ID
    uint8_t ocp_version;                         //!< OCP version
    uint8_t ocp_accelerator_management_msg_type; //!< Message Type
} __attribute__((packed));

struct Message
{
    BindingPciVid hdr; //!< OCP MCTP VDM message header
} __attribute__((packed));

struct BindingPciVidInfo
{
    uint8_t ocp_accelerator_management_msg_type;
    uint8_t instance_id;
    uint8_t msg_type;
};

struct CommonRequest
{
    Message msgHdr;
    uint8_t command;
    uint8_t data_size;
} __attribute__((packed));

struct CommonResponse
{
    Message msgHdr;
    uint8_t command;
    uint8_t completion_code;
    uint16_t reserved;
    uint16_t data_size;
} __attribute__((packed));

struct CommonAggregateResponse
{
    Message msgHdr;
    uint8_t command;
    uint8_t completion_code;
    uint16_t telemetryCount;
} __attribute__((packed));

struct CommonNonSuccessResponse
{
    Message msgHdr;
    uint8_t command;
    uint8_t completion_code;
    uint16_t reason_code;
} __attribute__((packed));

int packHeader(uint16_t pciVendorId, const BindingPciVidInfo& hdr,
               BindingPciVid& msg);

int packHeader(PackBuffer& buffer, uint16_t pciVendorId,
               MessageType ocpAcceleratorManagementMsgType, uint8_t instanceId,
               uint8_t msgType);

int unpackHeader(UnpackBuffer& buffer, uint16_t pciVendorId,
                 MessageType& ocpAcceleratorManagementMsgType,
                 uint8_t& instanceId, uint8_t& msgType);

int decodeReasonCodeAndCC(std::span<const uint8_t> buf, CompletionCode& cc,
                          uint16_t& reasonCode);

int unpackReasonCodeAndCC(UnpackBuffer& buffer, CompletionCode& cc,
                          uint16_t& reasonCode);

int decodeAggregateResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    std::move_only_function<int(const uint8_t tag, const uint8_t length,
                                const uint8_t* value)>
        handler);

} // namespace accelerator_management
} // namespace ocp
