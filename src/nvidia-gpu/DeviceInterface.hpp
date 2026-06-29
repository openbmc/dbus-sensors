/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <sdbusplus/message/native_types.hpp>

#include <cstdint>

class DeviceInterface
{
  public:
    DeviceInterface() = default;
    DeviceInterface(const DeviceInterface&) = delete;
    DeviceInterface(DeviceInterface&&) = delete;
    DeviceInterface& operator=(const DeviceInterface&) = delete;
    DeviceInterface& operator=(DeviceInterface&&) = delete;
    virtual ~DeviceInterface() = default;

    virtual void init() = 0;
    // Stop polling timer(s) + set reading sensors to unavailable/NaN.
    // D-Bus objects are retained.
    virtual void setOffline() = 0;
    // Resume polling.
    virtual void setOnline() = 0;
    // Re-target the device to a new MCTP EID (device re-enumerated with a
    // different endpoint ID). D-Bus objects are retained.
    virtual void setEid(uint8_t eid) = 0;
    // The D-Bus path of the EntityManager configuration object the
    // device was created from.
    virtual const sdbusplus::object_path& getPath() const = 0;
};
