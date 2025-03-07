/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "GpuTempSensor.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <boost/regex.hpp>
#include <sdbusplus/bus.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

using namespace std::literals;

static const boost::regex invalidDBusNameSubString{"[^a-zA-Z0-9._/]+"};
static std::string makeDBusNameValid(const std::string& name)
{
    return boost::regex_replace(name, invalidDBusNameSubString, "_");
}

GpuTempSensor::GpuTempSensor(sdbusplus::bus::bus& bus, mctp_eid_t eid,
                             uint8_t sensorId, const std::string& name,
                             const std::string& chassisPath, bool verbose) :
    eid(eid), sensorId(sensorId), verbose(verbose)

{
    auto sensorPath =
        "/xyz/openbmc_project/sensors/temperature/"s + makeDBusNameValid(name);
    valueIntf = std::make_unique<ValueIntf>(bus, sensorPath.c_str());
    valueIntf->unit(SensorUnit::DegreesC);

    associationIntf =
        std::make_unique<AssociationDefinitionsInft>(bus, sensorPath.c_str());
    std::vector<std::tuple<std::string, std::string, std::string>> associations;
    associations.emplace_back("chassis", "all_sensors", chassisPath);
    associationIntf->associations(associations);
}
