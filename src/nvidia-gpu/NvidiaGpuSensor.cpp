/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuSensor.hpp"

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
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

using namespace std::literals;

static constexpr double gpuTempSensorMaxReading = 127;
static constexpr double gpuTempSensorMinReading = -128;

NvidiaGpuTempSensor::NvidiaGpuTempSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& sensorConfiguration, const uint8_t eid, uint8_t sensorId,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData) :
    Sensor(escapeName(name), std::move(thresholdData), sensorConfiguration,
           "temperature", false, true, gpuTempSensorMaxReading,
           gpuTempSensorMinReading, conn),
    eid(eid), sensorId{sensorId}, mctpRequester(mctpRequester),
    objectServer(objectServer)
{
    std::string dbusPath =
        sensorPathPrefix + "temperature/"s + escapeName(name);

    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(dbusPath, interface);
    }

    association = objectServer.add_interface(dbusPath, association::interface);

    setInitialProperties(sensor_paths::unitDegreesC);

    if (sensorId == gpuTLimitSensorId)
    {
        sensorTypeInterface = objectServer.add_interface(
            dbusPath, "xyz.openbmc_project.Sensor.Type");

        sensorTypeInterface->register_property(
            "ReadingBasis",
            "xyz.openbmc_project.Sensor.Type.ReadingBasisType.Headroom"s);
        sensorTypeInterface->register_property(
            "Implementation",
            "xyz.openbmc_project.Sensor.Type.ImplementationType.Synthesized"s);

        if (!sensorTypeInterface->initialize())
        {
            lg2::error(
                "Error initializing Type Interface for Temperature Sensor for eid {EID} and sensor id {SID}",
                "EID", eid, "SID", sensorId);
        }
    }

    commonPhysicalContextInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Common.PhysicalContext");

    commonPhysicalContextInterface->register_property(
        "Type",
        "xyz.openbmc_project.Common.PhysicalContext.PhysicalContextType.GPU"s);

    if (!commonPhysicalContextInterface->initialize())
    {
        lg2::error(
            "Error initializing PhysicalContext Interface for Temperature Sensor for eid {EID} and sensor id {SID}",
            "EID", eid, "SID", sensorId);
    }
}

NvidiaGpuTempSensor::~NvidiaGpuTempSensor()
{
    for (const auto& iface : thresholdInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    objectServer.remove_interface(association);
    objectServer.remove_interface(sensorInterface);
    if (sensorTypeInterface)
    {
        objectServer.remove_interface(sensorTypeInterface);
    }
    if (commonPhysicalContextInterface)
    {
        objectServer.remove_interface(commonPhysicalContextInterface);
    }
}

void NvidiaGpuTempSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

void NvidiaGpuTempSensor::processResponse(const std::error_code& ec,
                                          std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error(
            "Error updating Temperature Sensor for eid {EID} and sensor id {SID} : sending message over MCTP failed, rc={RC}",
            "EID", eid, "SID", sensorId, "RC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    double tempValue = 0;

    auto rc = gpu::decodeGetTemperatureReadingResponse(buffer, cc, reasonCode,
                                                       tempValue);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating Temperature Sensor for eid {EID} and sensor id {SID} : decode failed. "
            "rc={RC}, cc={CC}, reasonCode={RESC}",
            "EID", eid, "SID", sensorId, "RC", rc, "CC", cc, "RESC",
            reasonCode);
        return;
    }

    updateValue(tempValue);
}

void NvidiaGpuTempSensor::update()
{
    auto rc = gpu::encodeGetTemperatureReadingRequest(
        0, sensorId, getTemperatureReadingRequest);

    if (rc != 0)
    {
        lg2::error(
            "Error updating Temperature Sensor for eid {EID} and sensor id {SID} : encode failed, rc={RC}",
            "EID", eid, "SID", sensorId, "RC", rc);
    }

    mctpRequester.sendRecvMsg(
        eid, getTemperatureReadingRequest,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaGpuTempSensor> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaGpuTempSensor");
                return;
            }
            self->processResponse(ec, buffer);
        });
}
