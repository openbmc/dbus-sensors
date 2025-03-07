/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "Thresholds.hpp"
#include "sensor.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/**
 * @struct DeviceInfo
 * @brief Contains information about a device
 */
struct DeviceInfo
{
    uint8_t deviceType;
    uint8_t instanceId;
};

/**
 * @struct GpuTempSensor
 * @brief Implements a GPU temperature sensor that monitors temperature values
 * @details Inherits from Sensor base class and enables shared pointer
 * management via std::enable_shared_from_this
 */
struct GpuTempSensor :
    public Sensor,
    public std::enable_shared_from_this<GpuTempSensor>
{
  public:
    /**
     * @brief Constructor for GpuTempSensor
     * @param conn D-Bus connection
     * @param io Boost ASIO I/O context for asynchronous operations
     * @param mctpRequester MCTP protocol requester for GPU communication
     * @param name Name of the sensor
     * @param sensorConfiguration Configuration string for the sensor
     * @param objectServer D-Bus object server
     * @param thresholdData Vector of threshold configurations
     * @param pollRate How often to poll for new readings
     * @param deviceInfo Information about the GPU device
     * @param verbose Whether to enable verbose logging
     */
    GpuTempSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                  boost::asio::io_context& io,
                  mctp::MctpRequester& mctpRequester, const std::string& name,
                  const std::string& sensorConfiguration,
                  sdbusplus::asio::object_server& objectServer,
                  std::vector<thresholds::Threshold>&& thresholdData,
                  std::chrono::milliseconds pollRate,
                  const DeviceInfo& deviceInfo, bool verbose = false);

    /**
     * @brief Destructor
     */
    ~GpuTempSensor() override;

    /**
     * @brief Check if any thresholds have been crossed
     * @details Overrides the base class method to implement GPU-specific
     * threshold checking
     */
    void checkThresholds() override;

  private:
    /**
     * @brief Read the current temperature value from the GPU
     */
    void read();

    /**
     * @brief Initialize the sensor
     */
    void init();

    /**
     * @brief Update the sensor reading
     */
    void update();

    /**
     * @brief Discover available GPUs on the system
     */
    void discoverGpus();

    /**
     * @brief Process a discovered GPU endpoint
     * @param eid The endpoint ID of the discovered GPU
     */
    void processGpuEndpoint(uint8_t eid);

    /**
     * @brief MCTP endpoint ID
     */
    uint8_t eid{};

    /**
     * @brief The sensor ID
     */
    uint8_t sensorId;

    /**
     * @brief How often to poll the sensor in milliseconds
     */
    std::chrono::milliseconds sensorPollMs;

    /**
     * @brief Whether to enable verbose logging
     */
    bool verbose;

    /**
     * @brief Information about the GPU device
     */
    DeviceInfo deviceInfo;

    /**
     * @brief Timer for scheduling sensor reads
     */
    boost::asio::steady_timer waitTimer;

    /**
     * @brief Reference to the MCTP requester for communication
     */
    mctp::MctpRequester& mctpRequester;

    /**
     * @brief D-Bus connection
     */
    std::shared_ptr<sdbusplus::asio::connection> conn;

    /**
     * @brief D-Bus object server
     */
    sdbusplus::asio::object_server& objectServer;
};

/**
 * @brief Create GPU temperature sensors
 * @param io Boost ASIO I/O context
 * @param objectServer D-Bus object server
 * @param sensors Map to store created sensors
 * @param dbusConnection D-Bus connection
 * @param mctpRequester MCTP requester for GPU communication
 */
void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuTempSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    mctp::MctpRequester& mctpRequester);
