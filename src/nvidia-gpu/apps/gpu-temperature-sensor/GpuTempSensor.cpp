/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "GpuTempSensor.hpp"

#include <bits/basic_string.h>
#include <nvidia_sensors.h>
#include <ocp_ami.h>
#include <transport.h>

#include <MctpRequester.hpp>
#include <boost/asio/awaitable.hpp>
#include <boost/regex.hpp>
#include <phosphor-logging/lg2.hpp>
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
    eid(eid),
    sensorId(sensorId), verbose(verbose)

{
    auto sensorPath = "/xyz/openbmc_project/sensors/temperature/"s +
                      makeDBusNameValid(name);
    valueIntf = std::make_unique<ValueIntf>(bus, sensorPath.c_str());
    valueIntf->unit(SensorUnit::DegreesC);

    associationIntf =
        std::make_unique<AssociationDefinitionsInft>(bus, sensorPath.c_str());
    std::vector<std::tuple<std::string, std::string, std::string>> associations;
    associations.emplace_back("chassis", "all_sensors", chassisPath);
    associationIntf->associations(associations);
}

boost::asio::awaitable<void>
    GpuTempSensor::update(mctp::MctpRequester& mctpRequester)
{
    std::vector<uint8_t> reqMsg(
        sizeof(struct ocp_ami_binding_pci_vid) +
        sizeof(ocp_ami_oem_nvidia_get_temperature_reading_req));

    auto* msg = reinterpret_cast<struct ocp_ami_msg*>(reqMsg.data());

    auto rc = ocp_ami_oem_nvidia_encode_get_temperature_reading_req(
        OCP_AMI_INSTANCE_MIN, sensorId, msg);

    // Disable clang-tidy check as the error is in header
    // boost/asio/impl/awaitable.hpp and to exclude that header from clang-tidy
    // checks, clang-tidy configuration option ExcludeHeaderFilterRegex is only
    // supported in clang-tidy version 19.

    // NOLINTNEXTLINE
    if (rc != 0)
    {
        lg2::error(
            "GpuTempSensor::update(): encode_get_temperature_reading_req failed, rc={RC}",
            "RC", rc);
        co_return;
    }

    std::vector<uint8_t> respMsg;
    std::tie(rc, respMsg) = co_await mctpRequester.sendRecvMsg(eid, reqMsg);

    if (rc != 0)
    {
        lg2::error(
            "GpuTempSensor::update(): MctpRequester::sendRecvMsg() failed, rc={RC}",
            "RC", rc);
        co_return;
    }

    uint8_t cc = 0;
    uint16_t reasonCode = 0;
    double tempValue = 0;

    if (respMsg.empty())
    {
        lg2::error(
            "GpuTempSensor::update(): MctpRequester::sendRecvMsg() failed, respMsgLen=0");
        co_return;
    }

    rc = ocp_ami_oem_nvidia_decode_get_temperature_reading_resp(
        reinterpret_cast<const ocp_ami_msg*>(respMsg.data()), respMsg.size(),
        &cc, &reasonCode, &tempValue);
    if (rc != 0)
    {
        lg2::error(
            "GpuTempSensor::update(): decode_get_temperature_reading_resp() failed, rc={RC} cc={CC} reasonCode={RESC}\n",
            "RC", rc, "CC", cc, "RESC", reasonCode);
        co_return;
    }

    valueIntf->value(tempValue);

    if (verbose)
    {
        lg2::info(
            "GpuTempSensor::update(): eid={EID} sensorId={SID} temp={TEMP}",
            "EID", eid, "SID", sensorId, "TEMP", tempValue);
    }

    co_return;
}
