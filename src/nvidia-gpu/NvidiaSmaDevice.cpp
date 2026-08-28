/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaSmaDevice.hpp"

#include "NvidiaGpuTempSensor.hpp"
#include "NvidiaSensorConfig.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

// What each temperature sensor id stands for, from the sensor table for VR
// products in the NSM MCU usage specification.
constexpr auto temperatureSensorNames =
    std::to_array<std::pair<uint8_t, const char*>>({
        {16, "SMA_Ext"},
        {17, "SMA_Internal"},
        {18, "NvLink"},
        {19, "BusBar"},
        {138, "GPU1_Die_A"},
        {139, "GPU1_Die_B"},
        {140, "GPU2_Die_A"},
        {141, "GPU2_Die_B"},
        {144, "PCB_1"},
        {145, "PCB_2"},
        {168, "HSCC"},
        {192, "HSC"},
        {216, "CPU1_Die"},
        {217, "CPU1_SoC"},
        {218, "CPU2_Die"},
        {219, "CPU2_SoC"},
    });

std::string temperatureSensorName(const std::string& deviceName,
                                  uint8_t sensorId)
{
    const auto* const named =
        std::ranges::find(temperatureSensorNames, sensorId,
                          &std::pair<uint8_t, const char*>::first);

    if (named == temperatureSensorNames.end())
    {
        return deviceName + "_TEMP_" + std::to_string(sensorId);
    }

    return deviceName + "_" + named->second;
}

SmaDevice::SmaDevice(const SensorConfigs& configs, const std::string& name,
                     const sdbusplus::object_path& path,
                     const std::shared_ptr<sdbusplus::asio::connection>& conn,
                     uint8_t eid, boost::asio::io_context& io,
                     mctp::MctpRequester& mctpRequester,
                     sdbusplus::asio::object_server& objectServer) :
    eid(eid), sensorPollMs(std::chrono::milliseconds{configs.pollRate}),
    waitTimer(io, std::chrono::steady_clock::duration(0)),
    mctpRequester(mctpRequester), conn(conn), objectServer(objectServer),
    configs(configs), name(escapeName(name)), path(path)
{
    const int rc = gpu::encodeGetTemperatureReadingRequest(
        0, gpu::temperatureAggregateSensorId, tempRequest);

    if (rc == 0)
    {
        tempRequestEncoded = true;
    }
    else
    {
        lg2::error(
            "Failed to encode temperature request for eid {EID}, rc={RC}",
            "EID", eid, "RC", rc);
    }
}

void SmaDevice::init()
{
    makeInventory();
    makeSensors();
}

void SmaDevice::makeSensors()
{
    lg2::info("Added MCA {NAME} Sensors with chassis path: {PATH}.", "NAME",
              name, "PATH", path);
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
    for (const auto& [sensorId, sensor] : tempSensors)
    {
        sensor->markFunctional(false);
    }
}

void SmaDevice::setOnline()
{
    setFunctional(true);
    for (const auto& [sensorId, sensor] : tempSensors)
    {
        sensor->markFunctional(true);
    }
    read();
}

void SmaDevice::setEid(uint8_t newEid)
{
    eid = newEid;
    for (const auto& [sensorId, sensor] : tempSensors)
    {
        sensor->setEid(newEid);
    }
}

void SmaDevice::read()
{
    updateTempSensors();

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

void SmaDevice::updateTempSensors()
{
    if (!tempRequestEncoded)
    {
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, tempRequest,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<SmaDevice> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid SmaDevice reference");
                return;
            }
            self->processTempSensorResponse(ec, buffer);
        });
}

void SmaDevice::processTempSensorResponse(const std::error_code& ec,
                                          std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error(
            "Error reading temperatures for eid {EID}: sending message over MCTP failed, rc={RC}",
            "EID", eid, "RC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;

    const int rc = gpu::decodeGetTemperatureReadingsResponse(
        buffer, cc, reasonCode, tempReadings);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error reading temperatures for eid {EID}: rc={RC}, cc={CC}, reasonCode={RESC}",
            "EID", eid, "RC", rc, "CC", cc, "RESC", reasonCode);
        return;
    }

    for (const auto& reading : tempReadings)
    {
        auto sensor = tempSensors.find(reading.sensorId);

        if (sensor == tempSensors.end())
        {
            sensor =
                tempSensors
                    .emplace(reading.sensorId,
                             std::make_shared<NvidiaGpuTempSensor>(
                                 conn, mctpRequester,
                                 temperatureSensorName(name, reading.sensorId),
                                 path, eid, reading.sensorId, objectServer,
                                 std::vector<thresholds::Threshold>{},
                                 gpu::DeviceIdentification::DEVICE_SMA))
                    .first;

            lg2::info("Added temperature sensor {ID} for {NAME}", "ID",
                      reading.sensorId, "NAME", name);
        }

        sensor->second->updateValue(reading.temperatureC);
    }
}
