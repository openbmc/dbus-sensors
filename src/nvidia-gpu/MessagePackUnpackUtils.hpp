/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <bit>
#include <cerrno>
#include <concepts>
#include <cstdint>
#include <cstring>
#include <span>

class PackBuffer
{
  public:
    PackBuffer(std::span<uint8_t> buffer) : buffer(buffer) {}

    template <typename T>
    int pack(T data)
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
    int unpack(T& data)
        requires std::integral<T>;

    int skip(size_t length);

    std::span<const uint8_t> getRemaining() const
    {
        return buffer;
    }

    int getError() const
    {
        return error;
    }

  private:
    std::span<const uint8_t> buffer;
    int error = 0;
};

template <typename T>
int PackBuffer::pack(T data)
    requires std::integral<T>
{
    if (error != 0)
    {
        return error;
    }

    if (buffer.size() < sizeof(T))
    {
        error = EINVAL;
        return error;
    }

    if constexpr (sizeof(T) > 1 && std::endian::native != std::endian::little)
    {
        data = std::byteswap(data);
    }

    std::memcpy(buffer.data(), &data, sizeof(T));

    buffer = buffer.subspan<sizeof(T)>();

    return 0;
}

template <typename T>
int UnpackBuffer::unpack(T& data)
    requires std::integral<T>
{
    if (error != 0)
    {
        return error;
    }

    if (buffer.size() < sizeof(T))
    {
        error = EINVAL;
        return error;
    }

    std::memcpy(&data, buffer.data(), sizeof(T));

    if constexpr (sizeof(T) > 1 && std::endian::native != std::endian::little)
    {
        data = std::byteswap(data);
    }

    buffer = buffer.subspan<sizeof(T)>();

    return 0;
}

inline int UnpackBuffer::skip(const size_t length)
{
    if (error != 0)
    {
        return error;
    }

    if (buffer.size() < length)
    {
        error = EINVAL;
        return error;
    }

    buffer = buffer.subspan(length);

    return 0;
}
