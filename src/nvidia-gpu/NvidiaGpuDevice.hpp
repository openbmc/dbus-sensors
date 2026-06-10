/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "Inventory.hpp"
#include "MctpRequester.hpp"
#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaEventReporting.hpp"
#include "NvidiaGpuClockFrequencyMetric.hpp"
#include "NvidiaGpuClockSpeedControl.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaGpuMemoryClockFrequency.hpp"
#include "NvidiaGpuMemoryDevice.hpp"
#include "NvidiaGpuPowerControl.hpp"
#include "NvidiaGpuPowerSensor.hpp"
#include "NvidiaGpuTempSensor.hpp"
#include "NvidiaGpuXid.hpp"

#include <NvidiaDriverInformation.hpp>
#include <NvidiaGpuEnergySensor.hpp>
#include <NvidiaGpuPowerPeakReading.hpp>
#include <NvidiaGpuUtilizationMetrics.hpp>
#include <NvidiaGpuViolationDuration.hpp>
#include <NvidiaGpuVoltageSensor.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <NvidiaPcieFunction.hpp>
#include <NvidiaPcieInterface.hpp>
#include <NvidiaPciePort.hpp>
#include <NvidiaPciePortMetrics.hpp>
#include <SerialQueue.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class GpuDevice : public std::enable_shared_from_this<GpuDevice>
{
  public:
    GpuDevice(const SensorConfigs& configs, const std::string& name,
              const std::string& path,
              const std::shared_ptr<sdbusplus::asio::connection>& conn,
              uint8_t eid, boost::asio::io_context& io,
              mctp::MctpRequester& mctpRequester,
              sdbusplus::asio::object_server& objectServer,
              const gpu::DeviceCapabilities& caps);

    ~GpuDevice();

    const std::string& getPath() const
    {
        return path;
    }

    void init();

  private:
    void makeSensors();

    void read();

    void readLongRunning();

    void processTLimitThresholds(const std::error_code& ec);

    void getTLimitThresholds();

    uint8_t eid{};

    void getNextThermalParameter();
    void readThermalParameterCallback(const std::error_code& ec,
                                      std::span<const uint8_t> buffer);

    std::chrono::milliseconds sensorPollMs;

    boost::asio::steady_timer waitTimer;

    boost::asio::steady_timer waitTimerLongRunning;

    mctp::MctpRequester& mctpRequester;

    boost::asio::io_context& io;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    sdbusplus::asio::object_server& objectServer;

    std::shared_ptr<NvidiaGpuTempSensor> tempSensor;
    std::shared_ptr<NvidiaGpuTempSensor> tLimitSensor;
    std::shared_ptr<NvidiaGpuTempSensor> dramTempSensor;
    std::shared_ptr<NvidiaGpuPowerSensor> powerSensor;
    std::shared_ptr<NvidiaGpuPowerPeakReading> peakPower;
    std::shared_ptr<NvidiaGpuEnergySensor> energySensor;
    std::shared_ptr<NvidiaGpuVoltageSensor> voltageSensor;
    std::shared_ptr<NvidiaDriverInformation> driverInfo;
    std::shared_ptr<NvidiaGpuPowerControl> gpuPowerControl;
    std::shared_ptr<sdbusplus::asio::dbus_interface> powerCapInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> dramAssociationInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> dramItemInterface;

    std::shared_ptr<NvidiaPcieInterface> pcieInterface;
    std::shared_ptr<NvidiaPciePortInfo> pciePort;
    std::shared_ptr<NvidiaPcieFunction> pcieFunction;
    std::vector<std::shared_ptr<NvidiaPciePortMetrics>> pciePortMetrics;
    std::shared_ptr<NvidiaGpuMemoryDevice> memoryDevice;
    std::shared_ptr<NvidiaGpuMemoryClockFrequency> memoryClockFrequency;

    std::shared_ptr<NvidiaEventReportingConfig> eventReporting;
    std::shared_ptr<SerialQueue> longRunningQueue;
    std::shared_ptr<NvidiaLongRunningResponseHandler> longRunningHandler;
    std::shared_ptr<NvidiaGpuUtilizationMetrics> utilizationMetrics;
    std::shared_ptr<NvidiaGpuViolationDuration> violationDuration;

    std::shared_ptr<NvidiaXidEventHandler> xidEventHandler;

    std::array<uint8_t, gpu::readThermalParametersRequestSize>
        thermalParamReqMsg{};
    std::array<int32_t, 3> thresholds{};
    size_t current_threshold_index{};

    SensorConfigs configs;

    std::string name;

    std::string path;

    std::shared_ptr<Inventory> inventory;

    std::shared_ptr<sdbusplus::asio::dbus_interface> controlClockSpeedInterface;
    std::shared_ptr<NvidiaGpuClockFrequencyMetric> clockFrequencyMetric;
    std::shared_ptr<NvidiaGpuClockSpeedControl> gpuClockSpeedControl;

    gpu::DeviceCapabilities caps;
};
