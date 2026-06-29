/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <cstdint>
#include <string>

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
    virtual const std::string& getPath() const = 0;
};
