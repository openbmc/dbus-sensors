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
#include <sdbusplus/message.hpp>

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";
constexpr const char* sensorType = "NvidiaMctpVdm";

using getSubTreeRet = std::vector<
    std::pair<std::string,
              std::vector<std::pair<std::string, std::vector<std::string>>>>>;
using GpuSensorConfigMap =
    std::map<std::string, std::variant<std::string, bool, uint32_t, uint8_t,
                                       int64_t, std::vector<uint8_t>>>;

struct GpuDevice
{
  public:
    /**
     * @brief Constructor for GpuDevice
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
    GpuDevice(const std::string& name, const std::string& path,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_context& io, mctp::MctpRequester& mctpRequester,
              sdbusplus::asio::object_server& objectServer);

    const std::string& getPath()
    {
        return path;
    }

  private:
    void addSensor(const std::string& name,
                   const std::shared_ptr<Sensor>& sensor);

    void createSensors();
    /**
     * @brief Read the current temperature value from the GPU
     */
    void read();

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
    void processMctpEndpoints(const boost::system::error_code& ec,
                              const getSubTreeRet& ret);

    /**
     * @brief Process configuration properties for MCTP endpoints
     *
     * @param[in] ec Error code from the D-Bus properties method call
     * @param[in] configs Map of configuration properties for the endpoint
     */
    void processEndpointConfigs(const boost::system::error_code& ec,
                                const GpuSensorConfigMap& configs);
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
     * @brief How often to poll the sensor in milliseconds
     */
    std::chrono::milliseconds sensorPollMs;

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

    std::vector<std::shared_ptr<Sensor>> sensors;

    std::string name;
    std::string path;
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
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevice,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    mctp::MctpRequester& mctpRequester);

/**
 * @brief Handle D-Bus interface removal events
 * @param message D-Bus message containing interface removal information
 * @param sensors Map of GPU temperature sensors to check for removal
 */
void interfaceRemoved(
    sdbusplus::message_t& message,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevice);
