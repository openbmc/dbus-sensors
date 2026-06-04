/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"

#include "MctpRequester.hpp"

#include <sys/types.h>

#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>

// Deliberately leaked (never deleted) so the io_uring service outlives every
// static at process exit; see the declaration in MockDbusHelper.hpp.
// NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
boost::asio::io_context& DbusMockTestBase::io =
    *(new boost::asio::io_context());
std::shared_ptr<sdbusplus::asio::connection> DbusMockTestBase::conn;
std::unique_ptr<sdbusplus::asio::object_server> DbusMockTestBase::objectServer;
std::unique_ptr<mctp::MctpRequester> DbusMockTestBase::mctpRequester;
pid_t DbusMockTestBase::daemonPid = -1;
bool DbusMockTestBase::initialized = false;
bool DbusMockTestBase::cleanupRegistered = false;
