/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaSmaDevice.hpp"

#include "NvidiaGpuTempSensor.hpp"
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
#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

SmaDevice::SmaDevice(const EntityDeviceConfig& config,
                     const std::shared_ptr<sdbusplus::asio::connection>& conn,
                     uint8_t eid, boost::asio::io_context& io,
                     mctp::MctpRequester& mctpRequester,
                     sdbusplus::asio::object_server& objectServer) :
    eid(eid), sensorPollMs(std::chrono::milliseconds{config.pollRate}),
    waitTimer(io, std::chrono::steady_clock::duration(0)),
    mctpRequester(mctpRequester), conn(conn), objectServer(objectServer),
    name(escapeName(config.name)), path(config.path)
{}

void SmaDevice::init()
{
    makeInventory();
    makeSensors();
}

void SmaDevice::makeSensors()
{
    tempSensor = std::make_shared<NvidiaGpuTempSensor>(
        conn, mctpRequester, name + "_TEMP_0", path, eid, smaTempSensorId,
        objectServer, std::vector<thresholds::Threshold>{},
        gpu::DeviceIdentification::DEVICE_SMA);

    initLeakSensors();

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
    tempSensor->markFunctional(false);
}

void SmaDevice::setOnline()
{
    setFunctional(true);
    tempSensor->markFunctional(true);
    read();
}

void SmaDevice::setEid(uint8_t newEid)
{
    eid = newEid;
    if (tempSensor)
    {
        tempSensor->setEid(newEid);
    }
}

void SmaDevice::read()
{
    tempSensor->update();

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
