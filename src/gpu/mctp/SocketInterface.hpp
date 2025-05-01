/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "types.hpp"

#include <concepts>
#include <memory>

namespace mctp
{

/**
 * @brief Concept for MCTP socket interfaces
 *
 * This concept defines the requirements for an MCTP socket interface.
 * An MCTP socket interface must provide methods for initializing the socket,
 * sending a request, and receiving a response.
 */
template <typename T>
concept SocketInterface =
    requires(T socket, eid_t eid, const Request& request) {
        { socket.init() } -> std::same_as<std::weak_ptr<SocketFd>>;
        { socket.send(eid, request) } -> std::same_as<bool>;
        {
            socket.recv()
        } -> std::same_as<std::pair<eid_t, std::unique_ptr<Response>>>;
    };

} // namespace mctp
