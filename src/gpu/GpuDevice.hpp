/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "Requester.hpp"
#include "UpdatableSensor.hpp"

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
constexpr const char* deviceType = "NvidiaMctpVdm";

using getSubTreeRet = std::vector<
    std::pair<std::string,
              std::vector<std::pair<std::string, std::vector<std::string>>>>>;
using GpuDeviceConfigMap =
    std::map<std::string, std::variant<std::string, bool, uint32_t, uint8_t,
                                       int64_t, std::vector<uint8_t>>>;

/**
 * @struct GpuDevice
 * @brief Represents a GPU device in the system
 * @details Manages the lifecycle of a GPU device including discovery, sensor
 * creation, communication, and monitoring. Handles MCTP protocol interactions
 * with the physical GPU hardware.
 */
struct GpuDevice
{
  public:
    /**
     * @brief Constructor for GpuDevice
     * @details Initializes a GPU device object with the provided parameters and
     *          starts the process of discovering available sensors on the
     * device
     *
     * @param name Name of the GPU device for identification
     * @param path D-Bus object path for this GPU device
     * @param conn D-Bus connection for system communication
     * @param io Boost ASIO I/O context for asynchronous operations
     * @param mctpRequester MCTP protocol requester for GPU communication
     * @param objectServer D-Bus object server for exposing interfaces
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
    /**
     * @brief Create sensors for this GPU device
     * @details Discovers and creates all available sensor types on this GPU
     */
    void makeSensors();

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
    void queryEndpoints(const boost::system::error_code& ec,
                        const getSubTreeRet& ret);

    /**
     * @brief Process configuration properties for MCTP endpoints
     *
     * @param[in] ec Error code from the D-Bus properties method call
     * @param[in] configs Map of configuration properties for the endpoint
     */
    void processEndpoint(const boost::system::error_code& ec,
                         const GpuDeviceConfigMap& endpoint);

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

    /**
     * @brief Collection of sensors associated with this GPU device
     * @details Stores all sensor objects created for this GPU
     */
    std::vector<std::shared_ptr<GpuSensor>> sensors;

    /**
     * @brief Name of this GPU device
     * @details Human-readable identifier for the GPU
     */
    std::string name;

    /**
     * @brief D-Bus object path for this GPU device
     * @details Path where this GPU device is exposed in the D-Bus object
     * hierarchy
     */
    std::string path;
};

/**
 * @brief Create GPU temperature sensors
 * @details Discovers and creates GPU devices and their associated sensors in
 * the system. This function is called at startup and whenever configuration
 * changes are detected.
 *
 * @param io Boost ASIO I/O context for scheduling asynchronous operations
 * @param objectServer D-Bus object server for exposing sensor interfaces
 * @param gpuDevice Map to store created GPU device objects, keyed by their
 * paths
 * @param dbusConnection D-Bus connection for system communication
 * @param mctpRequester MCTP requester for GPU communication protocol
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
 * @param gpuDevice Map of GPU devices to check for removal
 */
void interfaceRemoved(
    sdbusplus::message_t& message,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevice);
