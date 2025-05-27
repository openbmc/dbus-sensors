/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuVoltageSensor.hpp"

#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "UpdatableSensor.hpp"
#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using namespace std::literals;

// Reading from the device is in millivolts and unit set on the dbus is volts.
static constexpr double gpuVoltageSensorMaxReading =
    std::numeric_limits<uint64_t>::max() / 1000.0;
static constexpr double gpuVoltageSensorMinReading =
    std::numeric_limits<uint64_t>::min();

GpuVoltageSensor::GpuVoltageSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& sensorConfiguration, const uint8_t eid, uint8_t sensorId,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData) :
    GpuSensor(conn, mctpRequester, escapeName(name), sensorConfiguration,
              "voltage", gpuVoltageSensorMaxReading, gpuVoltageSensorMinReading,
              eid, sensorId, objectServer, std::move(thresholdData))
{
    setInitialProperties(sensor_paths::unitVolts);
}

void GpuVoltageSensor::processResponse(int sendRecvMsgResult,
                                       std::optional<std::vector<uint8_t>> resp)
{
    if (sendRecvMsgResult != 0)
    {
        lg2::error(
            "Error updating Volatage Sensor: sending message over MCTP failed, rc={RC}",
            "RC", sendRecvMsgResult);
        return;
    }

    if (!resp.has_value())
    {
        lg2::error("Error updating Volatage Sensor: empty response");
        return;
    }

    const auto& respMsg = resp.value();

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t voltageValue = 0;

    auto rc =
        gpu::decodeGetVoltageResponse(respMsg, cc, reasonCode, voltageValue);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating Volatage Sensor: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
            "RC", rc, "CC", cc, "RESC", reasonCode);
        return;
    }

    // Reading from the device is in millivolts and unit set on the dbus
    // is volts.
    updateValue(voltageValue / 1000.0);
}

void GpuVoltageSensor::update()
{
    std::vector<uint8_t> reqMsg(
        sizeof(ocp::accelerator_management::BindingPciVid) +
        sizeof(gpu::GetCurrentEnergyCounterRequest));

    auto rc = gpu::encodeGetVoltageRequest(0, sensorId, reqMsg);
    if (rc != 0)
    {
        lg2::error("Error updating Volatage Sensor: encode failed, rc={RC}",
                   "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, reqMsg, std::bind_front(&GpuVoltageSensor::processResponse, this));
}
