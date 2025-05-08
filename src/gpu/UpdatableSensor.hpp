/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "Thresholds.hpp"
#include "mctp/Requester.hpp"
#include "sensor.hpp"

#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";

class GpuSensor : public Sensor
{
  public:
    GpuSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
              mctp::MctpRequester& mctpRequester, const std::string& name,
              const std::string& sensorConfiguration,
              const std::string& objectType, double max, double min,
              uint8_t eid, uint8_t sensorId,
              sdbusplus::asio::object_server& objectServer,
              std::vector<thresholds::Threshold>&& thresholdData);

    ~GpuSensor() override;

    virtual void update() = 0;

    /**
     * @brief Check if any thresholds have been crossed
     * @details Overrides the base class method to implement GPU-specific
     * threshold checking
     */
    void checkThresholds() override;

  protected:
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
