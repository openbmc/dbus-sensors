/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "NvidiaGpuDevice.hpp"
#include "Utils.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";
constexpr const char* deviceType = "NvidiaMctpVdm";

struct DeviceDiscoveryManager
{
  public:
    /**
     * @brief Constructor for DeviceDiscoveryManager
     * @details Discover devices with the provided parameters
     *
     * @param name Name of the device for identification
     * @param path D-Bus object path for this device
     * @param conn D-Bus connection for system communication
     * @param io Boost ASIO I/O context for asynchronous operations
     * @param mctpRequester MCTP protocol requester for device communication
     * @param objectServer D-Bus object server for exposing interfaces
     */
    DeviceDiscoveryManager(const std::string& name, const std::string& path,
                           std::shared_ptr<sdbusplus::asio::connection>& conn,
                           boost::asio::io_context& io,
                           mctp::MctpRequester& mctpRequester,
                           sdbusplus::asio::object_server& objectServer);

    /**
     * @brief Get the configuration chassis path
     */
    const std::string& getPath()
    {
        return path;
    }

  private:
    /**
     * @brief Discover available devices on the system
     */
    void discoverDevices();

    /**
     * @brief Process response received for Query Device Identification command
     * from the device
     */
    void processQueryDeviceIdResponse(uint8_t eid, int sendRecvMsgResult,
                                      std::optional<std::vector<uint8_t>> resp);

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
     * @brief Send Query Device Identification request
     * @param eid The endpoint ID of the discovered device
     */
    void queryDeviceIdentification(uint8_t eid);

    boost::asio::io_context& io;

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
     * @brief Name of this device
     * @details Human-readable identifier
     */
    std::string name;

    /**
     * @brief D-Bus object path for this device
     * @details Path where this device is exposed in the D-Bus object
     * hierarchy
     */
    std::string path;

    /**
     * @brief Global map of GPU devices keyed by their paths
     * @details Stores all discovered GPU devices in the system for management
     *          and tracking throughout the application lifecycle
     */
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>
        gpuDevice;
};

/**
 * @brief Create device sensors
 * @details Discovers and creates devices and their associated sensors in
 * the system. This function is called at startup and whenever configuration
 * changes are detected.
 *
 * @param io Boost ASIO I/O context for scheduling asynchronous operations
 * @param objectServer D-Bus object server for exposing sensor interfaces
 * @param gpuDevice Map to store created device objects, keyed by their
 * paths
 * @param dbusConnection D-Bus connection for system communication
 * @param mctpRequester MCTP requester for device communication protocol
 */
void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<
        std::string, std::shared_ptr<DeviceDiscoveryManager>>& gpuDevice,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    mctp::MctpRequester& mctpRequester);

/**
 * @brief Handle D-Bus interface removal events
 * @param message D-Bus message containing interface removal information
 * @param gpuDevice Map of devices to check for removal
 */
void interfaceRemoved(
    sdbusplus::message_t& message,
    boost::container::flat_map<
        std::string, std::shared_ptr<DeviceDiscoveryManager>>& gpuDevice);
