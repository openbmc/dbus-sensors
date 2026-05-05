/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaSmaLeakSensor.hpp"

#include "NvidiaSensorUtils.hpp"
#include "NvidiaUtils.hpp"
#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
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

NvidiaSmaLeakSensor::NvidiaSmaLeakSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn, const std::string& name,
    const std::string& sensorConfiguration,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData,
    const gpu::DeviceIdentification deviceType) :
    Sensor(escapeName(name), std::move(thresholdData), sensorConfiguration,
           "voltage", false, true, smaLeakSensorMaxReading,
           smaLeakSensorMinReading, conn),
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
                "Error initializing PhysicalContext Interface for Leak Sensor for sensor {NAME}",
                "NAME", name);
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

NvidiaSmaLeakSensorCarrier::NvidiaSmaLeakSensorCarrier(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& sensorConfiguration, const uint8_t eid,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData,
    const gpu::DeviceIdentification deviceType) :
    conn(conn), mctpRequester(mctpRequester), name(name),
    sensorConfiguration(sensorConfiguration), eid(eid),
    objectServer(objectServer), thresholdData(std::move(thresholdData)),
    deviceType(deviceType)
{}

void NvidiaSmaLeakSensorCarrier::init()
{
    auto leakReq = std::make_shared<
        std::array<uint8_t, gpu::getLeakDetectionInfoRequestSize>>();
    gpu::encodeGetLeakDetectionInfoRequest(0, *leakReq);
    mctpRequester.sendRecvMsg(
        eid, *leakReq,
        [weak{weak_from_this()}, leakReq](const std::error_code& ec,
                                          std::span<const uint8_t> response) {
            auto self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid NvidiaSmaLeakSensorCarrier reference");
                return;
            }
            if (ec)
            {
                lg2::error(
                    "Transport error probing Leak Sensor {NAME}: ec={EC}",
                    "NAME", self->name, "EC", ec.message());
                return;
            }

            ocp::accelerator_management::CompletionCode cc{};
            uint16_t reasonCode = 0;
            self->parsedSensors.clear();

            auto rc = gpu::decodeGetLeakDetectionInfoResponse(
                response, cc, reasonCode, self->parsedSensors);

            if (rc != 0 ||
                cc != ocp::accelerator_management::CompletionCode::SUCCESS)
            {
                lg2::error(
                    "Error creating Leak Sensor: decoding GetLeakDetectionInfo response for {NAME} failed, rc={RC}, cc={CC}, reasonCode={RESC}",
                    "NAME", self->name, "RC", rc, "CC", cc, "RESC", reasonCode);
                return;
            }

            if (self->parsedSensors.empty())
            {
                lg2::error(
                    "Error creating Leak Sensor: decode success but no sensors returned for {NAME}",
                    "NAME", self->name);
                return;
            }

            for (size_t i = 0; i < self->parsedSensors.size(); ++i)
            {
                uint8_t sensorId = self->parsedSensors[i].sensorId;
                std::string sensorName =
                    self->name + "_" + std::to_string(sensorId);

                // Use dynamic thresholds from hardware if exactly 3 are
                // provided
                std::vector<thresholds::Threshold> sensorThresholds =
                    self->thresholdData;
                if (self->parsedSensors[i].thresholds.size() == 3)
                {
                    sensorThresholds.clear();
                    sensorThresholds.emplace_back(
                        thresholds::Level::CRITICAL, thresholds::Direction::LOW,
                        self->parsedSensors[i].thresholds[0] / 1000.0);
                    sensorThresholds.emplace_back(
                        thresholds::Level::WARNING, thresholds::Direction::LOW,
                        self->parsedSensors[i].thresholds[1] / 1000.0);
                    sensorThresholds.emplace_back(
                        thresholds::Level::CRITICAL,
                        thresholds::Direction::HIGH,
                        self->parsedSensors[i].thresholds[2] / 1000.0);
                }

                auto newSensor = std::make_shared<NvidiaSmaLeakSensor>(
                    self->conn, sensorName, self->sensorConfiguration,
                    self->objectServer, std::move(sensorThresholds),
                    self->deviceType);
                newSensor->updateValue(
                    self->parsedSensors[i].adcReadingMv / 1000.0);
                self->leakSensors[sensorId] = newSensor;
            }
            lg2::info("Added SMA Leak Sensors with chassis path: {PATH}.",
                      "PATH", self->sensorConfiguration);
        });
}

void NvidiaSmaLeakSensorCarrier::processResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
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
    parsedSensors.clear();

    auto rc = gpu::decodeGetLeakDetectionInfoResponse(buffer, cc, reasonCode,
                                                      parsedSensors);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating Leak Sensor: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
            "RC", rc, "CC", cc, "RESC", reasonCode);
        return;
    }

    if (parsedSensors.empty())
    {
        lg2::error(
            "Error updating Leak Sensor: decode success but no sensors returned");
        return;
    }

    for (const auto& sensorData : parsedSensors)
    {
        auto it = leakSensors.find(sensorData.sensorId);
        if (it != leakSensors.end())
        {
            // Reading from the device is in millivolts and unit set on the dbus
            // is volts.
            it->second->updateValue(sensorData.adcReadingMv / 1000.0);
        }
    }
}

void NvidiaSmaLeakSensorCarrier::update()
{
    if (leakSensors.empty())
    {
        return;
    }

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
            std::shared_ptr<NvidiaSmaLeakSensorCarrier> self = weak.lock();
            if (!self)
            {
                lg2::error("invalid reference to NvidiaSmaLeakSensorCarrier");
                return;
            }
            self->processResponse(ec, buffer);
        });
}
