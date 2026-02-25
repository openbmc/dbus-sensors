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
#include <NvidiaGpuEnergySensor.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaGpuPowerPeakReading.hpp>
#include <NvidiaGpuPowerSensor.hpp>
#include <NvidiaGpuSensor.hpp>
#include <NvidiaGpuVoltageSensor.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
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

GpuDevice::GpuDevice(const SensorConfigs& configs, const std::string& name,
                     const std::string& path,
                     const std::shared_ptr<sdbusplus::asio::connection>& conn,
                     uint8_t eid, boost::asio::io_context& io,
                     mctp::MctpRequester& mctpRequester,
                     sdbusplus::asio::object_server& objectServer) :
    eid(eid), sensorPollMs(std::chrono::milliseconds{configs.pollRate}),
    waitTimer(io, std::chrono::steady_clock::duration(0)),
    mctpRequester(mctpRequester), conn(conn), objectServer(objectServer),
    configs(configs), name(escapeName(name)), path(path)
{
    inventory = std::make_shared<Inventory>(
        conn, objectServer, name, mctpRequester,
        gpu::DeviceIdentification::DEVICE_GPU, eid, io);
}

void GpuDevice::init()
{
    makeSensors();
    inventory->init();
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

constexpr uint8_t graphicsClockId = 0x00;

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

    int rc = gpu::encodeGetCurrentClockFrequencyRequest(
        0, graphicsClockId,
        std::span<uint8_t>(clockFreqReqMsg.data(), clockFreqReqMsg.size()));
    if (rc == 0)
    {
        mctpRequester.sendRecvMsg(
            eid, clockFreqReqMsg,
            [weak{weak_from_this()}](const std::error_code& ec,
                                     std::span<const uint8_t> buffer) {
                std::shared_ptr<GpuDevice> self = weak.lock();
                if (!self)
                {
                    return;
                }
                if (ec)
                {
                    return;
                }
                ocp::accelerator_management::CompletionCode cc{};
                uint16_t reasonCode = 0;
                uint32_t clockFreqMHz = 0;
                int decodeRc = gpu::decodeGetCurrentClockFrequencyResponse(
                    buffer, cc, reasonCode, clockFreqMHz);
                if (decodeRc == 0 &&
                    cc == ocp::accelerator_management::CompletionCode::SUCCESS)
                {
                    self->inventory->setOperatingSpeed(clockFreqMHz);
                }
            });
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
