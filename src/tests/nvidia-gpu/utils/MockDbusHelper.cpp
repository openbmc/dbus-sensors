/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"

#include "MctpRequester.hpp"

#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>

boost::asio::io_context DbusMockTestBase::io;
std::shared_ptr<sdbusplus::asio::connection> DbusMockTestBase::conn;
std::unique_ptr<sdbusplus::asio::object_server> DbusMockTestBase::objectServer;
std::unique_ptr<mctp::MctpRequester> DbusMockTestBase::mctpRequester;
bool DbusMockTestBase::initialized = false;
