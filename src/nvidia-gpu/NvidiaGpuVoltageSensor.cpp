/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuVoltageSensor.hpp"

#include "NvidiaSensorUtils.hpp"
#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaDeviceDiscovery.hpp>
#include <NvidiaGpuMctpVdm.hpp>
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

static constexpr double gpuVoltageSensorMaxReading = 50;
static constexpr double gpuVoltageSensorMinReading =
    std::numeric_limits<uint32_t>::min();

NvidiaGpuVoltageSensor::NvidiaGpuVoltageSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& sensorConfiguration, const uint8_t eid, uint8_t sensorId,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData,
    gpu::DeviceIdentification deviceType) :
    Sensor(escapeName(name), std::move(thresholdData), sensorConfiguration,
           "energy", false, true, gpuVoltageSensorMaxReading,
           gpuVoltageSensorMinReading, conn),
    eid(eid), sensorId{sensorId}, mctpRequester(mctpRequester),
    objectServer(objectServer)
{
    std::string dbusPath = sensorPathPrefix + "voltage/"s + escapeName(name);

    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(dbusPath, interface);
    }

    association = objectServer.add_interface(dbusPath, association::interface);

    setInitialProperties(sensor_paths::unitVolts);

    auto physicalContext =
        nvidia_sensor_utils::deviceTypeToPhysicalContext(deviceType);

    if (physicalContext)
    {
        commonPhysicalContextInterface = objectServer.add_interface(
            dbusPath, "xyz.openbmc_project.Common.PhysicalContext");

        commonPhysicalContextInterface->register_property("Type",
                                                          *physicalContext);

        if (!commonPhysicalContextInterface->initialize())
        {
            lg2::error(
                "Error initializing PhysicalContext Interface for Voltage Sensor for eid {EID} and sensor id {SID}",
                "EID", eid, "SID", sensorId);
        }
    }
}

NvidiaGpuVoltageSensor::~NvidiaGpuVoltageSensor()
{
    for (const auto& iface : thresholdInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    objectServer.remove_interface(sensorInterface);
    objectServer.remove_interface(association);
    if (commonPhysicalContextInterface)
    {
        objectServer.remove_interface(commonPhysicalContextInterface);
    }
}

void NvidiaGpuVoltageSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

void NvidiaGpuVoltageSensor::processResponse(const std::error_code& ec,
                                             std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error(
            "Error updating Voltage Sensor: sending message over MCTP failed, rc={RC}",
            "RC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t voltageValue = 0;

    auto rc =
        gpu::decodeGetVoltageResponse(buffer, cc, reasonCode, voltageValue);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating Voltage Sensor: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
            "RC", rc, "CC", cc, "RESC", reasonCode);
        return;
    }

    // Reading from the device is in microvolts and unit set on the dbus
    // is volts.
    updateValue(voltageValue / 1000000.0);
}

void NvidiaGpuVoltageSensor::update()
{
    auto rc = gpu::encodeGetVoltageRequest(0, sensorId, request);

    if (rc != 0)
    {
        lg2::error("Error updating Voltage Sensor: encode failed, rc={RC}",
                   "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaGpuVoltageSensor> self = weak.lock();
            if (!self)
            {
                lg2::error("invalid reference to NvidiaGpuVoltageSensor");
                return;
            }
            self->processResponse(ec, buffer);
        });
}
