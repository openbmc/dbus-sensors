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
#include <NvidiaEventReporting.hpp>
#include <NvidiaGpuClockFrequencyMetric.hpp>
#include <NvidiaGpuClockSpeedControl.hpp>
#include <NvidiaGpuEnergySensor.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaGpuMemoryClockFrequency.hpp>
#include <NvidiaGpuMemoryDevice.hpp>
#include <NvidiaGpuPowerControl.hpp>
#include <NvidiaGpuPowerPeakReading.hpp>
#include <NvidiaGpuPowerSensor.hpp>
#include <NvidiaGpuTempSensor.hpp>
#include <NvidiaGpuUtilizationMetrics.hpp>
#include <NvidiaGpuViolationDuration.hpp>
#include <NvidiaGpuVoltageSensor.hpp>
#include <NvidiaGpuXid.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <NvidiaPcieFunction.hpp>
#include <NvidiaPcieInterface.hpp>
#include <NvidiaPciePort.hpp>
#include <NvidiaPciePortMetrics.hpp>
#include <NvidiaUtils.hpp>
#include <OcpMctpVdm.hpp>
#include <SerialQueue.hpp>
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
#include <functional>
#include <initializer_list>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

// Each long-running command can take up to 2s on timeout; worst-case
// capacity is ~15 commands/device.
static constexpr auto longRunningSensorPollRate = std::chrono::seconds{30};

static constexpr const char* controlClockSpeedIfaceName =
    "xyz.openbmc_project.Control.OperatingClockSpeed";
static constexpr const char* controlClockSpeedPrefix =
    "/xyz/openbmc_project/control/operatingclockspeed/";
static constexpr const char* controlPowerPrefix =
    "/xyz/openbmc_project/control/power/";

static constexpr uint8_t gpuTLimitCriticalThresholdId{1};
static constexpr uint8_t gpuTLimitWarningThresholdId{2};
static constexpr uint8_t gpuTLimitHardshutDownThresholdId{4};

// nota bene: the order has to match the order in processTLimitThresholds
static constexpr auto thresholdIds = std::to_array<uint8_t>(
    {gpuTLimitWarningThresholdId, gpuTLimitCriticalThresholdId,
     gpuTLimitHardshutDownThresholdId});

static constexpr const char* dramIfaceName =
    "xyz.openbmc_project.Inventory.Item.Dimm";

GpuDevice::GpuDevice(const SensorConfigs& configs, const std::string& name,
                     const std::string& path,
                     const std::shared_ptr<sdbusplus::asio::connection>& conn,
                     uint8_t eid, boost::asio::io_context& io,
                     mctp::MctpRequester& mctpRequester,
                     sdbusplus::asio::object_server& objectServer,
                     const gpu::DeviceCapabilities& caps) :
    eid(eid), sensorPollMs(std::chrono::milliseconds{configs.pollRate}),
    waitTimer(io, std::chrono::steady_clock::duration(0)),
    waitTimerLongRunning(io, std::chrono::steady_clock::duration(0)),
    mctpRequester(mctpRequester), io(io), conn(conn),
    objectServer(objectServer), configs(configs), name(escapeName(name)),
    path(path), caps(caps)
{
    const std::string powerControlPath = controlPowerPrefix + this->name;

    powerCapInterface = objectServer.add_interface(
        powerControlPath, "xyz.openbmc_project.Control.Power.Cap");

    powerCapInterface->register_property("MinPowerCapValue", uint32_t{0});
    powerCapInterface->register_property("MaxPowerCapValue",
                                         std::numeric_limits<uint32_t>::max());
    powerCapInterface->register_property("DefaultPowerCap",
                                         std::numeric_limits<uint32_t>::max());

    const sdbusplus::object_path gpuPath = inventoryPrefix / this->name;
    const sdbusplus::object_path dramPath =
        inventoryPrefix / std::format("{}_{}", this->name, dramInventorySuffix);

    std::vector<Association> associations;
    associations.emplace_back("contained_by", "containing", gpuPath);

    dramAssociationInterface =
        objectServer.add_interface(dramPath, association::interface);
    dramAssociationInterface->register_property("Associations", associations);

    if (!dramAssociationInterface->initialize())
    {
        lg2::error(
            "Error initializing DRAM association interface for {NAME}, eid={EID}",
            "NAME", this->name, "EID", eid);
    }

    dramItemInterface = objectServer.add_interface(dramPath, dramIfaceName);
    dramItemInterface->register_property(
        "MemoryType",
        std::string("xyz.openbmc_project.Inventory.Item.Dimm.DeviceType.HBM"));
    dramItemInterface->register_property(
        "ECC", std::string(
                   "xyz.openbmc_project.Inventory.Item.Dimm.Ecc.SingleBitECC"));
    dramItemInterface->register_property("MemorySizeInKB", size_t{0});
    dramItemInterface->register_property("MemoryConfiguredSpeedInMhz",
                                         uint16_t{0});
    dramItemInterface->register_property("AllowedSpeedsMT",
                                         std::vector<uint16_t>(2, 0));

    if (!dramItemInterface->initialize())
    {
        lg2::error(
            "Error initializing DRAM Item.Dimm interface for {NAME}, eid={EID}",
            "NAME", this->name, "EID", eid);
    }

    const std::string gpuClockSpeedControlPath =
        controlClockSpeedPrefix + this->name;
    controlClockSpeedInterface = objectServer.add_interface(
        gpuClockSpeedControlPath, controlClockSpeedIfaceName);
    controlClockSpeedInterface->register_property(
        "PresentSpeedLimitMaxHz", std::numeric_limits<uint64_t>::max(),
        sdbusplus::asio::PropertyPermission::readOnly);
    controlClockSpeedInterface->register_property(
        "PresentSpeedLimitMinHz", uint64_t{0},
        sdbusplus::asio::PropertyPermission::readOnly);
    controlClockSpeedInterface->register_property(
        "RequestedSpeedLimitMaxHz", std::numeric_limits<uint64_t>::max());
    controlClockSpeedInterface->register_property(
        "RequestedSpeedLimitMinHz", std::numeric_limits<uint64_t>::max());

    if (!controlClockSpeedInterface->initialize())
    {
        lg2::error(
            "Error initializing OperatingClockSpeed interface for {NAME}, eid={EID}",
            "NAME", this->name, "EID", eid);
    }
}

GpuDevice::~GpuDevice()
{
    objectServer.remove_interface(powerCapInterface);
    objectServer.remove_interface(dramAssociationInterface);
    objectServer.remove_interface(dramItemInterface);
    objectServer.remove_interface(controlClockSpeedInterface);
}

void GpuDevice::init()
{
    inventory = std::make_shared<Inventory>(
        conn, objectServer, name, mctpRequester,
        gpu::DeviceIdentification::DEVICE_GPU, eid, io, powerCapInterface,
        dramItemInterface);

    // GetInventoryInformation is a one-shot read (with retries) that runs
    // outside the poll loop, so gate it here as well: only read inventory
    // when the device reports the command as supported.
    if (caps.supports(
            gpu::PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION))
    {
        inventory->init();
    }

    makeSensors();

    eventReporting->init();
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

    longRunningQueue = std::make_shared<SerialQueue>(io);

    longRunningHandler = std::make_shared<NvidiaLongRunningResponseHandler>(io);

    xidEventHandler = std::make_shared<NvidiaXidEventHandler>(name, conn);

    eventReporting = std::make_shared<NvidiaEventReportingConfig>(
        eid, mctpRequester,
        std::initializer_list<EventDescriptor>{
            // Subscribe to the rediscovery event so the device pushes it when
            // its supported command codes change. This entry only sets the
            // event-source subscription mask; the handler that re-queries the
            // capabilities is registered by createDeviceForType.
            {gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY,
             static_cast<uint8_t>(
                 gpu::DeviceCapabilityDiscoveryEvents::REDISCOVERY),
             [](const EventInfo&, std::span<const uint8_t>) {}},
            {gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY,
             static_cast<uint8_t>(
                 gpu::DeviceCapabilityDiscoveryEvents::LONG_RUNNING_RESPONSE),
             std::bind_front(&NvidiaLongRunningResponseHandler::handler,
                             longRunningHandler)},
            {gpu::MessageType::PLATFORM_ENVIRONMENTAL,
             static_cast<uint8_t>(gpu::PlatformEnvironmentalEvent::XID),
             std::bind_front(&NvidiaXidEventHandler::handleXidEvent,
                             xidEventHandler)}});

    utilizationMetrics = std::make_shared<NvidiaGpuUtilizationMetrics>(
        mctpRequester, objectServer, name, eid, longRunningQueue,
        longRunningHandler);

    violationDuration = std::make_shared<NvidiaGpuViolationDuration>(
        mctpRequester, objectServer, name, eid, longRunningQueue,
        longRunningHandler);

    driverInfo = std::make_shared<NvidiaDriverInformation>(
        conn, mctpRequester, name, eid, objectServer,
        sdbusplus::object_path(path).parent_path());

    gpuPowerControl = std::make_shared<NvidiaGpuPowerControl>(
        objectServer, name, mctpRequester, eid, io, powerCapInterface,
        inventory);

    gpuClockSpeedControl = std::make_shared<NvidiaGpuClockSpeedControl>(
        objectServer, name, mctpRequester, eid, controlClockSpeedInterface);

    pcieInterface = std::make_shared<NvidiaPcieInterface>(
        conn, mctpRequester, name, path, eid, objectServer,
        gpu::DeviceIdentification::DEVICE_GPU);

    pciePort = std::make_shared<NvidiaPciePortInfo>(
        conn, mctpRequester, "UP_0", name, path, eid,
        gpu::PciePortType::UPSTREAM, 0, 0, objectServer,
        gpu::DeviceIdentification::DEVICE_GPU);

    pcieFunction = std::make_shared<NvidiaPcieFunction>(
        conn, mctpRequester, name, path, eid, 0, objectServer,
        gpu::DeviceIdentification::DEVICE_GPU);

    pciePortMetrics.emplace_back(makeNvidiaPciePortErrors(
        conn, mctpRequester, "UP_0", name, path, eid,
        gpu::PciePortType::UPSTREAM, 0, 0, objectServer,
        gpu::DeviceIdentification::DEVICE_GPU));

    pciePortMetrics.emplace_back(makeNvidiaPciePortCounters(
        conn, mctpRequester, "UP_0", name, path, eid,
        gpu::PciePortType::UPSTREAM, 0, 0, objectServer,
        gpu::DeviceIdentification::DEVICE_GPU));

    pciePortMetrics.emplace_back(makeNvidiaPciePortL0ToRecoveryCount(
        conn, mctpRequester, "UP_0", name, path, eid,
        gpu::PciePortType::UPSTREAM, 0, 0, objectServer,
        gpu::DeviceIdentification::DEVICE_GPU));

    memoryDevice = std::make_shared<NvidiaGpuMemoryDevice>(
        conn, mctpRequester, name, eid, objectServer);

    memoryClockFrequency = std::make_shared<NvidiaGpuMemoryClockFrequency>(
        mctpRequester, name, eid, dramItemInterface);

    clockFrequencyMetric = std::make_shared<NvidiaGpuClockFrequencyMetric>(
        mctpRequester, name, eid, objectServer);

    getTLimitThresholds();

    lg2::info("Added GPU {NAME} Sensors with chassis path: {PATH}.", "NAME",
              name, "PATH", path);
    read();
    readLongRunning();
}

void GpuDevice::getTLimitThresholds()
{
    if (!caps.supports(
            gpu::PlatformEnvironmentalCommands::READ_THERMAL_PARAMETERS))
    {
        processTLimitThresholds(std::make_error_code(std::errc::not_supported));
        return;
    }

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
        processTLimitThresholds(
            std::make_error_code(std::errc::protocol_error));
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
    using enum gpu::PlatformEnvironmentalCommands;

    // Sensors are always created; gate each poll on whether the device reports
    // the command it uses as supported. Commands the device does not report as
    // supported are simply not polled.
    if (caps.supports(GET_TEMPERATURE_READING))
    {
        tempSensor->update();
    }
    if (tLimitSensor)
    {
        if (caps.supports(GET_TEMPERATURE_READING))
        {
            tLimitSensor->update();
        }
    }
    if (caps.supports(GET_TEMPERATURE_READING))
    {
        dramTempSensor->update();
    }
    if (caps.supports(GET_CURRENT_POWER_DRAW))
    {
        powerSensor->update();
    }
    if (caps.supports(GET_MAX_OBSERVED_POWER))
    {
        peakPower->update();
    }
    if (caps.supports(GET_CURRENT_ENERGY_COUNTER))
    {
        energySensor->update();
    }
    if (caps.supports(GET_VOLTAGE))
    {
        voltageSensor->update();
    }
    if (caps.supports(GET_DRIVER_INFORMATION))
    {
        driverInfo->update();
    }
    if (caps.supports(GET_POWER_LIMITS))
    {
        gpuPowerControl->update();
    }
    if (caps.supports(GET_CLOCK_LIMIT))
    {
        gpuClockSpeedControl->update();
    }
    if (caps.supports(gpu::PcieLinkCommands::QueryScalarGroupTelemetryV1))
    {
        pcieInterface->update();
        pciePort->update();
        pcieFunction->update();
        for (auto& metrics : pciePortMetrics)
        {
            metrics->update();
        }
    }
    if (caps.supports(GET_ECC_ERROR_COUNTS))
    {
        memoryDevice->update();
    }
    if (caps.supports(GET_CURRENT_CLOCK_FREQUENCY))
    {
        memoryClockFrequency->update();
        clockFrequencyMetric->update();
    }

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

void GpuDevice::readLongRunning()
{
    if (caps.supports(
            gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION))
    {
        utilizationMetrics->update();
    }
    if (caps.supports(
            gpu::PlatformEnvironmentalCommands::GET_VIOLATION_DURATION))
    {
        violationDuration->update();
    }

    waitTimerLongRunning.expires_after(longRunningSensorPollRate);
    waitTimerLongRunning.async_wait(
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
            self->readLongRunning();
        });
}
