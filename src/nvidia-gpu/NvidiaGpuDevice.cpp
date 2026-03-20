/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuDevice.hpp"

#include "Thresholds.hpp"
#include "Utils.hpp"

#include <Inventory.hpp>
#include <MctpRequester.hpp>
#include <NvidiaDeviceDiscovery.hpp>
#include <NvidiaDriverInformation.hpp>
#include <NvidiaGpuControl.hpp>
#include <NvidiaGpuEnergySensor.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaGpuPowerPeakReading.hpp>
#include <NvidiaGpuPowerSensor.hpp>
#include <NvidiaGpuSensor.hpp>
#include <NvidiaGpuVoltageSensor.hpp>
#include <NvidiaPcieInterface.hpp>
#include <NvidiaPciePort.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

static constexpr uint8_t gpuTLimitCriticalThresholdId{1};
static constexpr uint8_t gpuTLimitWarningThresholdId{2};
static constexpr uint8_t gpuTLimitHardshutDownThresholdId{4};

// nota bene: the order has to match the order in processTLimitThresholds
static constexpr std::array<uint8_t, 3> thresholdIds{
    gpuTLimitWarningThresholdId, gpuTLimitCriticalThresholdId,
    gpuTLimitHardshutDownThresholdId};

static constexpr const char* controlPowerPrefix =
    "/xyz/openbmc_project/control/power/";

GpuDevice::GpuDevice(const SensorConfigs& configs, const std::string& name,
                     const std::string& path,
                     const std::shared_ptr<sdbusplus::asio::connection>& conn,
                     uint8_t eid, boost::asio::io_context& io,
                     mctp::MctpRequester& mctpRequester,
                     sdbusplus::asio::object_server& objectServer) :
    eid(eid), sensorPollMs(std::chrono::milliseconds{configs.pollRate}),
    waitTimer(io, std::chrono::steady_clock::duration(0)),
    mctpRequester(mctpRequester), io(io), conn(conn),
    objectServer(objectServer), configs(configs), name(escapeName(name)),
    path(path)
{
    const std::string powerControlPath = controlPowerPrefix + this->name;

    powerCapInterface = objectServer.add_interface(
        powerControlPath, "xyz.openbmc_project.Control.Power.Cap");

    powerCapInterface->register_property("PowerCap",
                                         std::numeric_limits<uint32_t>::max());
    powerCapInterface->register_property("PowerCapEnable", false);
    powerCapInterface->register_property("MinPowerCapValue", uint32_t{0});
    powerCapInterface->register_property("MaxPowerCapValue",
                                         std::numeric_limits<uint32_t>::max());
    powerCapInterface->register_property(
        "DefaultPowerCap", std::numeric_limits<uint32_t>::max(),
        sdbusplus::asio::PropertyPermission::readOnly);

    powerCapInterface->initialize();

    const std::string chassisPath =
        sdbusplus::message::object_path(this->path).parent_path();
    const std::string devicePath =
        chassisPath + "/" + this->name + "_Device_Assembly";
    const std::string boardPath =
        chassisPath + "/" + this->name + "_Board_Assembly";

    // Device Assembly
    deviceAssembly.panelIface = objectServer.add_interface(
        devicePath, "xyz.openbmc_project.Inventory.Item.Panel");
    deviceAssembly.panelIface->initialize();

    deviceAssembly.assetIface = objectServer.add_interface(
        devicePath, "xyz.openbmc_project.Inventory.Decorator.Asset");
    deviceAssembly.assetIface->register_property("Manufacturer",
                                                 std::string("NVIDIA"));
    deviceAssembly.assetIface->register_property("PartNumber", std::string{});
    deviceAssembly.assetIface->register_property("SerialNumber", std::string{});
    deviceAssembly.assetIface->register_property("Model", std::string{});
    deviceAssembly.assetIface->register_property("BuildDate", std::string{});
    deviceAssembly.assetIface->initialize();

    deviceAssembly.physicalContextIface = objectServer.add_interface(
        devicePath, "xyz.openbmc_project.Common.PhysicalContext");
    deviceAssembly.physicalContextIface->register_property(
        "Type", std::string("xyz.openbmc_project.Common.PhysicalContext."
                            "PhysicalContextType.Accelerator"));
    deviceAssembly.physicalContextIface->initialize();

    deviceAssembly.embeddedIface = objectServer.add_interface(
        devicePath, "xyz.openbmc_project.Inventory.Connector.Embedded");
    deviceAssembly.embeddedIface->initialize();

    deviceAssembly.itemIface = objectServer.add_interface(
        devicePath, "xyz.openbmc_project.Inventory.Item");
    deviceAssembly.itemIface->register_property("Present", true);
    deviceAssembly.itemIface->initialize();

    deviceAssembly.operationalStatusIface = objectServer.add_interface(
        devicePath, "xyz.openbmc_project.State.Decorator.OperationalStatus");
    deviceAssembly.operationalStatusIface->register_property(
        "Functional", true);
    deviceAssembly.operationalStatusIface->initialize();

    std::vector<Association> deviceAssociations;
    deviceAssociations.emplace_back("contained_by", "containing", chassisPath);
    deviceAssembly.associationIface =
        objectServer.add_interface(devicePath, association::interface);
    deviceAssembly.associationIface->register_property("Associations",
                                                       deviceAssociations);
    deviceAssembly.associationIface->initialize();

    // Board Assembly
    boardAssembly.panelIface = objectServer.add_interface(
        boardPath, "xyz.openbmc_project.Inventory.Item.Panel");
    boardAssembly.panelIface->initialize();

    boardAssembly.assetIface = objectServer.add_interface(
        boardPath, "xyz.openbmc_project.Inventory.Decorator.Asset");
    boardAssembly.assetIface->register_property("Manufacturer",
                                                std::string("NVIDIA"));
    boardAssembly.assetIface->register_property("PartNumber", std::string{});
    boardAssembly.assetIface->initialize();

    boardAssembly.physicalContextIface = objectServer.add_interface(
        boardPath, "xyz.openbmc_project.Common.PhysicalContext");
    boardAssembly.physicalContextIface->register_property(
        "Type", std::string("xyz.openbmc_project.Common.PhysicalContext."
                            "PhysicalContextType.Accelerator"));
    boardAssembly.physicalContextIface->initialize();

    boardAssembly.embeddedIface = objectServer.add_interface(
        boardPath, "xyz.openbmc_project.Inventory.Connector.Embedded");
    boardAssembly.embeddedIface->initialize();

    boardAssembly.itemIface = objectServer.add_interface(
        boardPath, "xyz.openbmc_project.Inventory.Item");
    boardAssembly.itemIface->register_property("Present", true);
    boardAssembly.itemIface->initialize();

    boardAssembly.operationalStatusIface = objectServer.add_interface(
        boardPath, "xyz.openbmc_project.State.Decorator.OperationalStatus");
    boardAssembly.operationalStatusIface->register_property("Functional", true);
    boardAssembly.operationalStatusIface->initialize();

    std::vector<Association> boardAssociations;
    boardAssociations.emplace_back("contained_by", "containing", chassisPath);
    boardAssembly.associationIface =
        objectServer.add_interface(boardPath, association::interface);
    boardAssembly.associationIface->register_property("Associations",
                                                      boardAssociations);
    boardAssembly.associationIface->initialize();
}

GpuDevice::~GpuDevice()
{
    objectServer.remove_interface(powerCapInterface);
    objectServer.remove_interface(deviceAssembly.panelIface);
    objectServer.remove_interface(deviceAssembly.assetIface);
    objectServer.remove_interface(deviceAssembly.physicalContextIface);
    objectServer.remove_interface(deviceAssembly.embeddedIface);
    objectServer.remove_interface(deviceAssembly.itemIface);
    objectServer.remove_interface(deviceAssembly.operationalStatusIface);
    objectServer.remove_interface(deviceAssembly.associationIface);
    objectServer.remove_interface(boardAssembly.panelIface);
    objectServer.remove_interface(boardAssembly.assetIface);
    objectServer.remove_interface(boardAssembly.physicalContextIface);
    objectServer.remove_interface(boardAssembly.embeddedIface);
    objectServer.remove_interface(boardAssembly.itemIface);
    objectServer.remove_interface(boardAssembly.operationalStatusIface);
    objectServer.remove_interface(boardAssembly.associationIface);
}

void GpuDevice::init()
{
    inventory = std::make_shared<Inventory>(
        conn, objectServer, name, mctpRequester,
        gpu::DeviceIdentification::DEVICE_GPU, eid, io, powerCapInterface,
        deviceAssembly.assetIface, boardAssembly.assetIface);

    inventory->init();

    makeSensors();
}

void GpuDevice::makeSensors()
{
    tempSensor = std::make_shared<NvidiaGpuTempSensor>(
        conn, mctpRequester, name + "_TEMP_0", path, eid, gpuTempSensorId,
        objectServer, std::vector<thresholds::Threshold>{},
        gpu::DeviceIdentification::DEVICE_GPU);

    dramTempSensor = std::make_shared<NvidiaGpuTempSensor>(
        conn, mctpRequester, name + "_DRAM_0_TEMP_0", path, eid,
        gpuDramTempSensorId, objectServer,
        std::vector<thresholds::Threshold>{thresholds::Threshold{
            thresholds::Level::CRITICAL, thresholds::Direction::HIGH, 95.0}},
        gpu::DeviceIdentification::DEVICE_GPU);

    powerSensor = std::make_shared<NvidiaGpuPowerSensor>(
        conn, mctpRequester, name + "_Power_0", path, eid, gpuPowerSensorId,
        objectServer, std::vector<thresholds::Threshold>{},
        gpu::DeviceIdentification::DEVICE_GPU);

    peakPower = std::make_shared<NvidiaGpuPowerPeakReading>(
        mctpRequester, name + "_Power_0", eid, gpuPeakPowerSensorId,
        objectServer);

    energySensor = std::make_shared<NvidiaGpuEnergySensor>(
        conn, mctpRequester, name + "_Energy_0", path, eid, gpuEnergySensorId,
        objectServer, std::vector<thresholds::Threshold>{},
        gpu::DeviceIdentification::DEVICE_GPU);

    voltageSensor = std::make_shared<NvidiaGpuVoltageSensor>(
        conn, mctpRequester, name + "_Voltage_0", path, eid, gpuVoltageSensorId,
        objectServer, std::vector<thresholds::Threshold>{},
        gpu::DeviceIdentification::DEVICE_GPU);

    driverInfo = std::make_shared<NvidiaDriverInformation>(
        conn, mctpRequester, name, path, eid, objectServer);

    gpuControl = std::make_shared<NvidiaGpuControl>(
        objectServer, name, inventoryPrefix + name, mctpRequester, eid,
        powerCapInterface);

    pcieInterface = std::make_shared<NvidiaPcieInterface>(
        conn, mctpRequester, name, path, eid, objectServer,
        gpu::DeviceIdentification::DEVICE_GPU);

    pciePort = std::make_shared<NvidiaPciePortInfo>(
        conn, mctpRequester, "UP_0", name, path, eid,
        gpu::PciePortType::UPSTREAM, 0, 0, objectServer,
        gpu::DeviceIdentification::DEVICE_GPU);

    getTLimitThresholds();

    lg2::info("Added GPU {NAME} Sensors with chassis path: {PATH}.", "NAME",
              name, "PATH", path);
    read();
}

void GpuDevice::getTLimitThresholds()
{
    thresholds = {};
    current_threshold_index = 0;
    getNextThermalParameter();
}

void GpuDevice::readThermalParameterCallback(const std::error_code& ec,
                                             std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error(
            "Error reading thermal parameter: sending message over MCTP failed, rc={RC}",
            "RC", ec.message());
        processTLimitThresholds(ec);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    int32_t threshold = 0;

    int rc = gpu::decodeReadThermalParametersResponse(buffer, cc, reasonCode,
                                                      threshold);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error reading thermal parameter: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
            "RC", rc, "CC", cc, "RESC", reasonCode);
        processTLimitThresholds(ec);
        return;
    }

    thresholds[current_threshold_index] = threshold;

    current_threshold_index++;

    if (current_threshold_index < thresholdIds.size())
    {
        getNextThermalParameter();
        return;
    }
    processTLimitThresholds(std::error_code{});
}

void GpuDevice::getNextThermalParameter()
{
    uint8_t id = thresholdIds[current_threshold_index];
    auto rc =
        gpu::encodeReadThermalParametersRequest(0, id, thermalParamReqMsg);
    if (rc != 0)
    {
        lg2::error(
            "Error reading thermal parameter for eid {EID} and parameter id {PID} : encode failed. rc={RC}",
            "EID", eid, "PID", id, "RC", rc);
        processTLimitThresholds(
            std::make_error_code(static_cast<std::errc>(rc)));
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, thermalParamReqMsg,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<GpuDevice> self = weak.lock();
            if (!self)
            {
                lg2::error("Failed to get lock on GpuDevice");
                return;
            }
            self->readThermalParameterCallback(ec, buffer);
        });
}

void GpuDevice::processTLimitThresholds(const std::error_code& ec)
{
    std::vector<thresholds::Threshold> tLimitThresholds{};
    if (!ec)
    {
        tLimitThresholds = {
            thresholds::Threshold{thresholds::Level::WARNING,
                                  thresholds::Direction::LOW,
                                  static_cast<double>(thresholds[0])},
            thresholds::Threshold{thresholds::Level::CRITICAL,
                                  thresholds::Direction::LOW,
                                  static_cast<double>(thresholds[1])},
            thresholds::Threshold{thresholds::Level::HARDSHUTDOWN,
                                  thresholds::Direction::LOW,
                                  static_cast<double>(thresholds[2])}};
    }

    tLimitSensor = std::make_shared<NvidiaGpuTempSensor>(
        conn, mctpRequester, name + "_TEMP_1", path, eid, gpuTLimitSensorId,
        objectServer, std::move(tLimitThresholds),
        gpu::DeviceIdentification::DEVICE_GPU);
}

void GpuDevice::read()
{
    tempSensor->update();
    if (tLimitSensor)
    {
        tLimitSensor->update();
    }
    dramTempSensor->update();
    powerSensor->update();
    peakPower->update();
    energySensor->update();
    voltageSensor->update();
    driverInfo->update();
    gpuControl->update();
    pcieInterface->update();
    pciePort->update();

    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait(
        [weak{weak_from_this()}](const boost::system::error_code& ec) {
            std::shared_ptr<GpuDevice> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to GpuDevice");
                return;
            }
            if (ec)
            {
                return;
            }
            self->read();
        });
}
