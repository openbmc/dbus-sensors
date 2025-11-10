/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>

#include <concepts>
#include <cstdint>
#include <cstring>
#include <span>

class PackBuffer
{
  public:
    PackBuffer(std::span<uint8_t> buffer) : buffer(buffer) {}

    template <typename T>
    void pack(const T& data)
        requires std::integral<T>;

    template <typename T>
    void unpack(T& data)
        requires std::integral<T>;

    int getError() const
    {
        return error;
    }

  private:
    std::span<uint8_t> buffer;
    int error = 0;
};

class UnpackBuffer
{
  public:
    UnpackBuffer(std::span<const uint8_t> buffer) : buffer(buffer) {}

    template <typename T>
    void unpack(T& data)
        requires std::integral<T>;

    int getError() const
    {
        return error;
    }

  private:
    std::span<const uint8_t> buffer;
    int error = 0;
};

template <typename T>
void PackBuffer::pack(const T& data)
    requires std::integral<T>
{
    if (error != 0)
    {
        return;
    }

    if (buffer.size() < sizeof(T))
    {
        error = EINVAL;
        return;
    }

    std::memcpy(buffer.data(), &data, sizeof(T));
    buffer = buffer.subspan<sizeof(T)>();
}

template <typename T>
void UnpackBuffer::unpack(T& data)
    requires std::integral<T>
{
    if (error != 0)
    {
        return;
    }

    if (buffer.size() < sizeof(T))
    {
        error = EINVAL;
        return;
    }

    std::memcpy(&data, buffer.data(), sizeof(T));
    buffer = buffer.subspan<sizeof(T)>();
}

inline int encodeRequestCommonHeader(PackBuffer& buffer,
                                     gpu::MessageType msgType, uint8_t command,
                                     uint8_t instanceId)
{
    int rc = ocp::accelerator_management::packHeader(
        buffer, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::REQUEST, instanceId,
        static_cast<uint8_t>(msgType));

    if (rc != 0)
    {
        return rc;
    }

    buffer.pack(command);

    return 0;
}

inline int decodeResponseCommonHeader(
    UnpackBuffer& buffer, gpu::MessageType msgType, uint8_t command,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode)
{
    ocp::accelerator_management::MessageType receivedMsgType{};
    uint8_t instanceId = 0;
    uint8_t receivedMessageType = 0;

    int rc = ocp::accelerator_management::unpackHeader(
        buffer, gpu::nvidiaPciVendorId, receivedMsgType, instanceId,
        receivedMessageType);

    if (rc != 0)
    {
        return rc;
    }

    if (receivedMsgType != ocp::accelerator_management::MessageType::RESPONSE)
    {
        return EINVAL;
    }

    if (receivedMessageType != static_cast<uint8_t>(msgType))
    {
        return EINVAL;
    }

    uint8_t receivedCommand = 0;
    buffer.unpack(receivedCommand);

    if (command != receivedCommand)
    {
        return EINVAL;
    }

    rc = ocp::accelerator_management::unpackReasonCodeAndCC(
        buffer, cc, reasonCode);

    if (rc != 0)
    {
        return rc;
    }

    return 0;
}
