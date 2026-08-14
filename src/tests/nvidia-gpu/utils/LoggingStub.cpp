/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "LoggingStub.hpp"

#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <map>
#include <memory>
#include <string>

namespace logging_stub
{

namespace
{

constexpr const char* loggingService = "xyz.openbmc_project.Logging";
constexpr const char* loggingPath = "/xyz/openbmc_project/logging";
constexpr const char* loggingCreateIface = "xyz.openbmc_project.Logging.Create";

CreateCall* activeCall = nullptr;

// Kept for the life of the binary: the interface has to stay registered, and
// a well-known name cannot be given back once claimed.
std::shared_ptr<sdbusplus::asio::dbus_interface>& stubInterface()
{
    static std::shared_ptr<sdbusplus::asio::dbus_interface> iface;
    return iface;
}

} // namespace

void setActiveCall(CreateCall* call)
{
    activeCall = call;
}

bool ensure(const std::shared_ptr<sdbusplus::asio::connection>& conn,
            sdbusplus::asio::object_server& objectServer)
{
    if (stubInterface())
    {
        return true;
    }

    auto iface = objectServer.add_interface(loggingPath, loggingCreateIface);
    iface->register_method(
        "Create", [](const std::string& message, const std::string& severity,
                     const std::map<std::string, std::string>& additionalData) {
            if (activeCall == nullptr)
            {
                return;
            }
            activeCall->count++;
            activeCall->message = message;
            activeCall->severity = severity;
            activeCall->additionalData = additionalData;
        });

    if (!iface->initialize())
    {
        return false;
    }

    conn->request_name(loggingService);
    stubInterface() = iface;
    return true;
}

} // namespace logging_stub
