/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "Thresholds.hpp"
#include "UpdatableSensor.hpp"

#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

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
    public GpuSensor,
    public std::enable_shared_from_this<GpuTempSensor>
{
  public:
    /**
     * @brief Constructor for GpuTempSensor
     * @param conn D-Bus connection for system communication
     * @param mctpRequester MCTP protocol requester for GPU communication
     * @param name Name of the sensor for identification in the system
     * @param sensorConfiguration Configuration string for the sensor containing
     * setup parameters
     * @param eid EID of the device endpoint
     * @param objectServer D-Bus object server for exposing sensor interfaces
     * @param thresholdData Vector of threshold configurations for temperature
     * monitoring
     */
    GpuTempSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                  mctp::MctpRequester& mctpRequester, const std::string& name,
                  const std::string& sensorConfiguration, uint8_t eid,
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
     * @brief Update the sensor reading
     */
    void update() final;

    /**
     * @brief MCTP endpoint ID
     */
    uint8_t eid{};

    /**
     * @brief The sensor ID
     */
    uint8_t sensorId;

    /**
     * @brief Reference to the MCTP requester for communication
     */
    mctp::MctpRequester& mctpRequester;

    /**
     * @brief D-Bus object server
     */
    sdbusplus::asio::object_server& objectServer;
};
