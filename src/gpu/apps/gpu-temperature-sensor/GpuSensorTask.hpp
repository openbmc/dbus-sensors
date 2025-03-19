/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <boost/asio/awaitable.hpp>
#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/connection.hpp>

#include <memory>

/**
 * @brief Main task for handling GPU sensor updates
 *
 * This function is the main entry point for the GPU sensor task.
 * It sets up the necessary components and handles the sensor updates.
 *
 * @param ctx Reference to the Boost ASIO I/O context.
 * @param dbusConnection Reference to the D-Bus connection.
 * @param verbose Flag to enable verbose logging.
 * @return Awaitable void.
 */
boost::asio::awaitable<void> gpuSensorTask(
    boost::asio::io_context& ctx,
    std::shared_ptr<sdbusplus::asio::connection> dbusConnection,
    bool verbose = false);
