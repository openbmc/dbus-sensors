/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <map>
#include <memory>
#include <string>

/**
 * Stub xyz.openbmc_project.Logging.Create for tests whose code under test
 * creates phosphor-logging entries.
 *
 * The object path and the well-known name can each be claimed only once on a
 * connection, and every test in the binary shares one connection, so the stub
 * has to be registered exactly once no matter how many fixtures need it. Each
 * fixture then routes the captured arguments to itself, the same link-seam
 * idea as mock_mctp::setActiveMock.
 */
namespace logging_stub
{

// Arguments of the Logging.Create calls seen while a capture was active.
struct CreateCall
{
    int count{0};
    std::string message;
    std::string severity;
    std::map<std::string, std::string> additionalData;
};

// Register the stub unless it is already up. False if the interface could not
// be initialized, in which case no Create call can ever be captured.
bool ensure(const std::shared_ptr<sdbusplus::asio::connection>& conn,
            sdbusplus::asio::object_server& objectServer);

// Route subsequent Create() calls into `call`; pass nullptr to stop.
void setActiveCall(CreateCall* call);

} // namespace logging_stub
