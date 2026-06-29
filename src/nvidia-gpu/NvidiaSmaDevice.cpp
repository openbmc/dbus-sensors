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
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

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
