/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "UpdatableSensor.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

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
     * @param eid GPU EID
     * @param io Boost ASIO I/O context for asynchronous operations
     * @param mctpRequester MCTP protocol requester for GPU communication
     * @param objectServer D-Bus object server for exposing interfaces
     */
    GpuDevice(const std::string& name, const std::string& path,
              std::shared_ptr<sdbusplus::asio::connection>& conn, uint8_t eid,
              boost::asio::io_context& io, mctp::MctpRequester& mctpRequester,
              sdbusplus::asio::object_server& objectServer);

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
