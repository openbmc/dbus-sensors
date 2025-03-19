/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <transport.h>

#include <boost/asio/awaitable.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/server/object.hpp>
#include <xyz/openbmc_project/Association/Definitions/server.hpp>
#include <xyz/openbmc_project/Sensor/Value/server.hpp>

#include <cstdint>
#include <memory>
#include <string>

using ValueIntf = sdbusplus::server::object_t<
    sdbusplus::xyz::openbmc_project::Sensor::server::Value>;
using SensorUnit = sdbusplus::xyz::openbmc_project::Sensor::server::Value::Unit;
using AssociationDefinitionsInft = sdbusplus::server::object_t<
    sdbusplus::xyz::openbmc_project::Association::server::Definitions>;

class GpuTempSensor
{
  public:
    GpuTempSensor(sdbusplus::bus::bus& bus, mctp_eid_t eid, uint8_t sensorId,
                  const std::string& name, const std::string& path,
                  bool verbose = false);

    boost::asio::awaitable<void> update(mctp::MctpRequester& mctpRequester);

  protected:
    mctp_eid_t eid;
    uint8_t sensorId;
    std::unique_ptr<ValueIntf> valueIntf;
    std::unique_ptr<AssociationDefinitionsInft> associationIntf;

    bool verbose;
};
