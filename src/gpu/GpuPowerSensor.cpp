/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "GpuPowerSensor.hpp"

#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "UpdatableSensor.hpp"
#include "Utils.hpp"
#include "mctp/Requester.hpp"

#include <GpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using namespace std::literals;

constexpr uint8_t gpuPowerSensorId{0};
/**
 * @brief GPU Power Sensor Averaging Interval in seconds, 0 implies default
 */
constexpr uint8_t gpuPowerAveragingIntervalInSec{0};
static constexpr double gpuPowerSensorMaxReading =
    std::numeric_limits<uint32_t>::max();
static constexpr double gpuPowerSensorMinReading =
    std::numeric_limits<uint32_t>::min();

GpuPowerSensor::GpuPowerSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& sensorConfiguration, uint8_t eid,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData) :
    GpuSensor(conn, mctpRequester, escapeName(name), sensorConfiguration,
              "power", gpuPowerSensorMaxReading, gpuPowerSensorMinReading, eid,
              gpuPowerSensorId, objectServer, std::move(thresholdData)),
    averagingInterval{gpuPowerAveragingIntervalInSec}
{
    setInitialProperties(sensor_paths::unitWatts);
}

void GpuPowerSensor::update()
{
    std::vector<uint8_t> reqMsg(
        sizeof(ocp::accelerator_management::BindingPciVid) +
        sizeof(gpu::GetCurrentPowerDrawRequest));

    auto rc = gpu::encodeGetCurrentPowerDrawRequest(0, sensorId,
                                                    averagingInterval, reqMsg);
    if (rc != 0)
    {
        lg2::error("Error updating Power Sensor: encode failed, rc={RC}", "RC",
                   rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, reqMsg,
        [this](int sendRecvMsgResult,
               std::optional<std::vector<uint8_t>> resp) {
            if (sendRecvMsgResult != 0)
            {
                lg2::error(
                    "Error updating Power Sensor: sending message over MCTP failed, rc={RC}",
                    "RC", sendRecvMsgResult);
                return;
            }

            if (!resp.has_value())
            {
                lg2::error("Error updating Power Sensor: empty response");
                return;
            }

            const auto& respMsg = resp.value();

            ocp::accelerator_management::CompletionCode cc{};
            uint16_t reasonCode = 0;
            uint32_t power = 0;

            auto rc = gpu::decodeGetCurrentPowerDrawResponse(respMsg, cc,
                                                             reasonCode, power);

            if (rc != 0 ||
                cc != ocp::accelerator_management::CompletionCode::SUCCESS)
            {
                lg2::error(
                    "Error updating Power Sensor: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
                    "RC", rc, "CC", cc, "RESC", reasonCode);
                return;
            }

            // Reading from the device is in milliwatts and unit set on the dbus
            // is watts.
            updateValue(power / 1000.0);
        });
}
