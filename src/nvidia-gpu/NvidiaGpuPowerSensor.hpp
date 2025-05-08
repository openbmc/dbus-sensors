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
#include <optional>
#include <string>
#include <vector>

/**
 * @struct GpuPowerSensor
 * @brief Implements a GPU power sensor that monitors power values
 * @details Inherits from Sensor base class and enables shared pointer
 * management via std::enable_shared_from_this
 */
struct GpuPowerSensor :
    public GpuSensor,
    public std::enable_shared_from_this<GpuPowerSensor>
{
  public:
    /**
     * @brief Constructor for GpuPowerSensor
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
    GpuPowerSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                   mctp::MctpRequester& mctpRequester, const std::string& name,
                   const std::string& sensorConfiguration, uint8_t eid,
                   sdbusplus::asio::object_server& objectServer,
                   std::vector<thresholds::Threshold>&& thresholdData);

  private:
    /**
     * @brief Update the sensor reading
     */
    void update() final;

    /**
     * @brief Process response received from the device
     */
    void processResponse(int sendRecvMsgResult,
                         std::optional<std::vector<uint8_t>> resp);

    /**
     * @brief The sensor ID
     */
    uint8_t averagingInterval;
};
