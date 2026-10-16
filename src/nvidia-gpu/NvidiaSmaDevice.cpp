/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaSmaDevice.hpp"

#include "NvidiaGpuPowerSensor.hpp"
#include "NvidiaGpuTempSensor.hpp"
#include "NvidiaGpuVoltageSensor.hpp"
#include "NvidiaSensorConfig.hpp"
#include "NvidiaSmaLeakSensor.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

SmaDevice::SmaDevice(
    const NvidiaMctpVdmConfig& config, const SmaDeviceConfigs& smaConfig,
    const std::shared_ptr<sdbusplus::asio::connection>& conn, uint8_t eid,
    boost::asio::io_context& io, mctp::MctpRequester& mctpRequester,
    sdbusplus::asio::object_server& objectServer) :
    eid(eid), sensorPollMs(std::chrono::milliseconds{config.pollRate}),
    waitTimer(io, std::chrono::steady_clock::duration(0)),
    mctpRequester(mctpRequester), conn(conn), objectServer(objectServer),
    smaConfig(smaConfig), name(escapeName(config.name)), path(config.path)
{}

void SmaDevice::init()
{
    makeInventory();
    makeSensors();
}

void SmaDevice::makeSensors()
{
    for (size_t index = 0; index < smaConfig.sensors.size(); ++index)
    {
        const SmaSensorConfig& sensor = smaConfig.sensors[index];
        if (sensor.dynamicThresholds.empty())
        {
            publishSensor(sensor);
        }
        else
        {
            readDynamicThresholds(index);
        }
    }

    initLeakSensors();

    lg2::info("Added MCA {NAME} Sensors with chassis path: {PATH}.", "NAME",
              name, "PATH", path);
}

void SmaDevice::readDynamicThresholds(size_t index)
{
    const SmaSensorConfig& sensor = smaConfig.sensors[index];
    auto request = std::make_shared<
        std::array<uint8_t, gpu::readThermalParametersRequestSize>>();
    const int rc =
        gpu::encodeReadThermalParametersRequest(0, sensor.id, *request);
    if (rc != 0)
    {
        lg2::error(
            "{NAME}: failed to encode the thermal parameter request for sensor {SID}, rc={RC}, publishing {SENSOR} without its dynamic thresholds",
            "NAME", name, "SID", sensor.id, "RC", rc, "SENSOR", sensor.name);
        publishSensor(sensor);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, *request,
        [weak{weak_from_this()}, index, request](
            const std::error_code& ec, std::span<const uint8_t> response) {
            auto self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to SmaDevice");
                return;
            }
            self->processDynamicThresholdsResponse(index, ec, response);
        });
}

void SmaDevice::processDynamicThresholdsResponse(
    size_t index, const std::error_code& ec, std::span<const uint8_t> response)
{
    SmaSensorConfig& sensor = smaConfig.sensors[index];
    if (ec)
    {
        lg2::error(
            "{NAME}: reading the thermal parameter of sensor {SID} failed: {ERROR}, publishing {SENSOR} without its dynamic thresholds",
            "NAME", name, "SID", sensor.id, "ERROR", ec.message(), "SENSOR",
            sensor.name);
        publishSensor(sensor);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    int32_t value = 0;
    const int rc = gpu::decodeReadThermalParametersResponse(
        response, cc, reasonCode, value);
    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "{NAME}: reading the thermal parameter of sensor {SID} failed, rc={RC}, cc={CC}, reasonCode={RESC}, publishing {SENSOR} without its dynamic thresholds",
            "NAME", name, "SID", sensor.id, "RC", rc, "CC", cc, "RESC",
            reasonCode, "SENSOR", sensor.name);
        publishSensor(sensor);
        return;
    }

    for (thresholds::Threshold threshold : sensor.dynamicThresholds)
    {
        threshold.value = static_cast<double>(value);
        sensor.thresholds.push_back(threshold);
    }
    publishSensor(sensor);
}

void SmaDevice::publishSensor(const SmaSensorConfig& sensor)
{
    const std::string sensorName = std::format("{}_{}", name, sensor.name);
    switch (sensor.kind)
    {
        case SmaSensorKind::Temperature:
            tempSensors.emplace_back(std::make_shared<NvidiaGpuTempSensor>(
                conn, mctpRequester, sensorName, path, eid, sensor.id,
                objectServer,
                std::vector<thresholds::Threshold>(sensor.thresholds),
                gpu::DeviceIdentification::DEVICE_SMA));
            break;
        case SmaSensorKind::Power:
            powerSensors.emplace_back(std::make_shared<NvidiaGpuPowerSensor>(
                conn, mctpRequester, sensorName, path, eid, sensor.id,
                objectServer, std::vector<thresholds::Threshold>{},
                gpu::DeviceIdentification::DEVICE_SMA));
            break;
        case SmaSensorKind::Voltage:
            voltageSensors.emplace_back(
                std::make_shared<NvidiaGpuVoltageSensor>(
                    conn, mctpRequester, sensorName, path, eid, sensor.id,
                    objectServer, std::vector<thresholds::Threshold>{},
                    gpu::DeviceIdentification::DEVICE_SMA));
            break;
    }
}

void SmaDevice::makeInventory()
{
    inventoryPath =
        (sdbusplus::object_path("/xyz/openbmc_project/inventory") / name);

    itemInterface = objectServer.add_interface(
        inventoryPath, "xyz.openbmc_project.Inventory.Item");
    if (!itemInterface->initialize())
    {
        lg2::error("Error initializing Item interface for {NAME}, eid={EID}",
                   "NAME", name, "EID", eid);
    }

    operationalStatusInterface = objectServer.add_interface(
        inventoryPath, "xyz.openbmc_project.State.Decorator.OperationalStatus");
    operationalStatusInterface->register_property("Functional", false);
    if (!operationalStatusInterface->initialize())
    {
        lg2::error(
            "Error initializing OperationalStatus interface for {NAME}, eid={EID}",
            "NAME", name, "EID", eid);
    }

    // The configuration object lives under the board entity-manager created,
    // so its parent is the board this device is on.
    std::vector<Association> associations;
    associations.emplace_back("contained_by", "containing", path.parent_path());

    associationInterface =
        objectServer.add_interface(inventoryPath, association::interface);
    associationInterface->register_property("Associations", associations);
    if (!associationInterface->initialize())
    {
        lg2::error(
            "Error initializing Association interface for {NAME}, eid={EID}",
            "NAME", name, "EID", eid);
    }
}

void SmaDevice::setFunctional(bool functional)
{
    if (operationalStatusInterface)
    {
        operationalStatusInterface->set_property("Functional", functional);
    }
}

void SmaDevice::setOffline()
{
    setFunctional(false);
    waitTimer.cancel();
    for (const auto& sensor : tempSensors)
    {
        sensor->markFunctional(false);
    }
    for (const auto& sensor : powerSensors)
    {
        sensor->markFunctional(false);
    }
    for (const auto& sensor : voltageSensors)
    {
        sensor->markFunctional(false);
    }
    for (auto& sensor : leakSensors)
    {
        sensor->markFunctional(false);
    }
}

void SmaDevice::setOnline()
{
    setFunctional(true);
    for (const auto& sensor : tempSensors)
    {
        sensor->markFunctional(true);
    }
    for (const auto& sensor : powerSensors)
    {
        sensor->markFunctional(true);
    }
    for (const auto& sensor : voltageSensors)
    {
        sensor->markFunctional(true);
    }
    for (auto& sensor : leakSensors)
    {
        sensor->markFunctional(true);
    }
    read();
}

void SmaDevice::setEid(uint8_t newEid)
{
    eid = newEid;
    for (const auto& sensor : tempSensors)
    {
        sensor->setEid(newEid);
    }
    for (const auto& sensor : powerSensors)
    {
        sensor->setEid(newEid);
    }
    for (const auto& sensor : voltageSensors)
    {
        sensor->setEid(newEid);
    }
    for (auto& sensor : leakSensors)
    {
        sensor->setEid(newEid);
    }
}

void SmaDevice::read()
{
    for (const auto& sensor : tempSensors)
    {
        sensor->update();
    }
    for (const auto& sensor : powerSensors)
    {
        sensor->update();
    }
    for (const auto& sensor : voltageSensors)
    {
        sensor->update();
    }

    for (auto& sensor : leakSensors)
    {
        sensor->update();
    }

    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait(
        [weak{weak_from_this()}](const boost::system::error_code& ec) {
            std::shared_ptr<SmaDevice> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid SmaDevice reference");
                return;
            }
            if (ec)
            {
                return;
            }
            self->read();
        });
}

void SmaDevice::initLeakSensors()
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
                lg2::error("Invalid reference to SmaDevice");
                return;
            }
            self->processLeakSensorsResponse(ec, response);
        });
}

void SmaDevice::processLeakSensorsResponse(const std::error_code& ec,
                                           std::span<const uint8_t> response)
{
    if (ec)
    {
        lg2::error(
            "Error creating Leak Sensor for {NAME}: sending message over MCTP failed, rc={RC}",
            "NAME", name, "RC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    std::vector<gpu::LeakSensorData> parsedSensors;

    auto rc = gpu::decodeGetLeakDetectionInfoResponse(response, cc, reasonCode,
                                                      parsedSensors);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error creating Leak Sensor: decoding GetLeakDetectionInfo response for {NAME} failed, rc={RC}, cc={CC}, reasonCode={RESC}",
            "NAME", name, "RC", rc, "CC", cc, "RESC", reasonCode);
        return;
    }

    if (parsedSensors.empty())
    {
        lg2::error(
            "Error creating Leak Sensor: decode success but no sensors returned for {NAME}",
            "NAME", name);
        return;
    }

    for (auto& parsedSensor : parsedSensors)
    {
        uint8_t sensorId = parsedSensor.sensorId;
        std::string sensorName =
            std::format("{}_LEAKDETECTOR_{}", name, sensorId);

        // Use dynamic thresholds from hardware if available
        std::vector<thresholds::Threshold> sensorThresholds;
        if (parsedSensor.thresholds.size() >= 2)
        {
            // Index 0: Min leak threshold
            sensorThresholds.emplace_back(thresholds::Level::CRITICAL,
                                          thresholds::Direction::LOW,
                                          parsedSensor.thresholds[0] / 1000.0);
            // Index 1: Max leak threshold
            sensorThresholds.emplace_back(thresholds::Level::WARNING,
                                          thresholds::Direction::LOW,
                                          parsedSensor.thresholds[1] / 1000.0);
        }
        if (parsedSensor.thresholds.size() >= 3)
        {
            // Index 2: Max normal threshold
            sensorThresholds.emplace_back(thresholds::Level::CRITICAL,
                                          thresholds::Direction::HIGH,
                                          parsedSensor.thresholds[2] / 1000.0);
        }

        auto newSensor = std::make_shared<NvidiaSmaLeakSensor>(
            conn, mctpRequester, sensorName, path.str, eid, sensorId,
            objectServer, std::move(sensorThresholds),
            gpu::DeviceIdentification::DEVICE_SMA);
        newSensor->updateValue(parsedSensor.adcReadingMv / 1000.0);
        leakSensors.emplace_back(std::move(newSensor));
    }
}
