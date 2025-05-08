/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuSensor.hpp"

#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "UpdatableSensor.hpp"
#include "Utils.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

using namespace std::literals;

static constexpr double gpuTempSensorMaxReading = 127;
static constexpr double gpuTempSensorMinReading = -128;

GpuTempSensor::GpuTempSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& sensorConfiguration, const uint8_t eid, uint8_t sensorId,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData,
    const std::string_view description) :
    GpuSensor(conn, mctpRequester, escapeName(name), sensorConfiguration,
              "temperature", gpuTempSensorMaxReading, gpuTempSensorMinReading,
              eid, sensorId, objectServer, std::move(thresholdData))
{
    std::string dbusPath =
        sensorPathPrefix + "temperature/"s + escapeName(name);

    if (!description.empty())
    {
        descriptionInterface = objectServer.add_interface(
            dbusPath, "xyz.openbmc_project.Inventory.Item");

        descriptionInterface->register_property("PrettyName",
                                                description.data());

        descriptionInterface->initialize();
    }

    setInitialProperties(sensor_paths::unitDegreesC);
}

GpuTempSensor::~GpuTempSensor()
{
    if (descriptionInterface)
    {
        objectServer.remove_interface(descriptionInterface);
    }
}

void GpuTempSensor::processResponse(int sendRecvMsgResult,
                                    std::optional<std::vector<uint8_t>> resp)
{
    if (sendRecvMsgResult != 0)
    {
        lg2::error(
            "Error updating Temperature Sensor: sending message over MCTP failed, rc={RC}",
            "RC", sendRecvMsgResult);
        return;
    }

    if (!resp.has_value())
    {
        lg2::error("Error updating Temperature Sensor: empty response");
        return;
    }

    const auto& respMsg = resp.value();

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    double tempValue = 0;

    auto rc = gpu::decodeGetTemperatureReadingResponse(respMsg, cc, reasonCode,
                                                       tempValue);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating Temperature Sensor: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
            "RC", rc, "CC", cc, "RESC", reasonCode);
        return;
    }

    updateValue(tempValue);
}

void GpuTempSensor::update()
{
    std::vector<uint8_t> reqMsg(
        sizeof(ocp::accelerator_management::BindingPciVid) +
        sizeof(gpu::GetTemperatureReadingRequest));

    auto rc = gpu::encodeGetTemperatureReadingRequest(0, sensorId, reqMsg);
    if (rc != 0)
    {
        lg2::error("Error updating Temperature Sensor: encode failed, rc={RC}",
                   "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, reqMsg, std::bind_front(&GpuTempSensor::processResponse, this));
}
