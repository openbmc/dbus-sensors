/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message.hpp>

#include <memory>
#include <string>
#include <vector>

constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";
constexpr const char* sensorType = "NvidiaMctpVdm";

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
                  boost::asio::io_context& io, const std::string& name,
                  const std::string& sensorConfiguration,
                  sdbusplus::asio::object_server& objectServer,
                  std::vector<thresholds::Threshold>&& thresholdData);

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
     * @brief Discover available GPUs on the system
     */
    void discoverGpus();

    /**
     * @brief Process MCTP endpoints discovered on the system
     *
     * @param[in] ec Error code from the D-Bus method call
     * @param[in] ret Object tree results containing MCTP endpoint information
     */
    void queryEndpoints(const boost::system::error_code& ec,
                        const GetSubTreeType& ret);

    /**
     * @brief Process configuration properties for MCTP endpoints
     *
     * @param[in] ec Error code from the D-Bus properties method call
     * @param[in] configs Map of configuration properties for the endpoint
     */
    void processEndpoint(const boost::system::error_code& ec,
                         const SensorBaseConfigMap& endpoint);

    /**
     * @brief Timer for scheduling sensor reads
     */
    boost::asio::steady_timer waitTimer;

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
 */
void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuTempSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection);

/**
 * @brief Handle D-Bus interface removal events
 * @param message D-Bus message containing interface removal information
 * @param sensors Map of GPU temperature sensors to check for removal
 */
void interfaceRemoved(
    sdbusplus::message_t& message,
    boost::container::flat_map<std::string, std::shared_ptr<GpuTempSensor>>&
        sensors);
