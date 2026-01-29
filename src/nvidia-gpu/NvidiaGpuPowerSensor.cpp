/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuPowerSensor.hpp"

#include "MctpRequester.hpp"
#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <bits/basic_string.h>

#include <NvidiaDeviceDiscovery.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaMetricReport.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

using namespace std::literals;

static constexpr double gpuPowerSensorMaxReading = 5000;
static constexpr double gpuPowerSensorMinReading =
    std::numeric_limits<uint32_t>::min();

NvidiaGpuPowerSensor::NvidiaGpuPowerSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& sensorConfiguration, uint8_t eid, uint8_t sensorId,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData,
    SensorMetricReport& sensorMetricReport) :
    Sensor(escapeName(name), std::move(thresholdData), sensorConfiguration,
           "power", false, true, gpuPowerSensorMaxReading,
           gpuPowerSensorMinReading, conn),
    eid(eid), sensorId{sensorId},

    mctpRequester(mctpRequester), objectServer(objectServer)

{
    std::string dbusPath = sensorPathPrefix + "power/"s + escapeName(name);

    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(dbusPath, interface);
    }

    association = objectServer.add_interface(dbusPath, association::interface);

    setInitialProperties(sensor_paths::unitWatts);

    sensorMetricReport.addSensor(
        escapeName(name), "power",
        sdbusplus::message::object_path{sensorConfiguration}
            .parent_path()
            .filename());
}

NvidiaGpuPowerSensor::~NvidiaGpuPowerSensor()
{
    for (const auto& iface : thresholdInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    objectServer.remove_interface(association);
    objectServer.remove_interface(sensorInterface);
}

void NvidiaGpuPowerSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

void NvidiaGpuPowerSensor::processResponse(const std::error_code& ec,
                                           std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error(
            "Error updating Power Sensor for eid {EID} and sensor id {SID} : sending message over MCTP failed, rc={RC}",
            "EID", eid, "SID", sensorId, "RC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t power = 0;

    const int rc =
        gpu::decodeGetPowerDrawResponse(buffer, cc, reasonCode, power);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating Power Sensor eid {EID} and sensor id {SID} : decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
            "EID", eid, "SID", sensorId, "RC", rc, "CC", cc, "RESC",
            reasonCode);
        return;
    }

    // Reading from the device is in milliwatts and unit set on the dbus
    // is watts.
    updateValue(power / 1000.0);
}

void NvidiaGpuPowerSensor::update()
{
    const int rc = gpu::encodeGetPowerDrawRequest(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_POWER_DRAW, 0, sensorId,
        averagingInterval, request);

    if (rc != 0)
    {
        lg2::error(
            "Error updating Temperature Sensor for eid {EID} and sensor id {SID} : encode failed, rc={RC}",
            "EID", eid, "SID", sensorId, "RC", rc);
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaGpuPowerSensor> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaGpuPowerSensor");
                return;
            }
            self->processResponse(ec, buffer);
        });
}
