/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaSmaLeakSensor.hpp"

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
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

using namespace std::literals;

static constexpr double smaLeakSensorMaxReading = 5;
static constexpr double smaLeakSensorMinReading = 0;
;

NvidiaSmaLeakSensor::NvidiaSmaLeakSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& sensorConfiguration, const uint8_t eid, uint8_t sensorId,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData,
    const gpu::DeviceIdentification deviceType) :
    Sensor(escapeName(name), std::move(thresholdData), sensorConfiguration,
           "energy", false, true, smaLeakSensorMaxReading,
           smaLeakSensorMinReading, conn),
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

    const std::optional<std::string> physicalContext =
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
                "Error initializing PhysicalContext Interface for Leak Sensor for eid {EID} and sensor id {SID}",
                "EID", eid, "SID", sensorId);
        }
    }
}

NvidiaSmaLeakSensor::~NvidiaSmaLeakSensor()
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

void NvidiaSmaLeakSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

void NvidiaSmaLeakSensor::processResponse(const std::error_code& ec,
                                          std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error(
            "Error updating Leak Sensor: sending message over MCTP failed, rc={RC}",
            "RC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    std::vector<gpu::LeakSensorData> parsedSensors;

    auto rc = gpu::decodeGetLeakDetectionInfoResponse(buffer, cc, reasonCode,
                                                      parsedSensors);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating Leak Sensor: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
            "RC", rc, "CC", cc, "RESC", reasonCode);
        return;
    }

    // Reading from the device is in microvolts and unit set on the dbus
    // is volts.
    for (const auto& sensor : parsedSensors)
    {
        if (sensor.sensorId == sensorId)
        {
            updateValue(sensor.adcReadingMv / 1000.0);
            break;
        }
    }
    updateValue(parsedSensors[0].adcReadingMv / 1000.0);
}

void NvidiaSmaLeakSensor::update()
{
    auto rc = gpu::encodeGetLeakDetectionInfoRequest(0, request);

    if (rc != 0)
    {
        lg2::error("Error updating Leak Sensor: encode failed, rc={RC}", "RC",
                   rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaSmaLeakSensor> self = weak.lock();
            if (!self)
            {
                lg2::error("invalid reference to NvidiaSmaLeakSensor");
                return;
            }
            self->processResponse(ec, buffer);
        });
}
