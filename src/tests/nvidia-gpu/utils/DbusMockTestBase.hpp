/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/system/error_code.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/asio/property.hpp>
#include <sdbusplus/exception.hpp>

#include <cerrno>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <gtest/gtest.h>

/**
 * Owns the private dbus-daemon and the shared bus objects for the whole
 * binary. Bring-up failure is not fatal: DbusMockTestBase fixtures skip
 * themselves, so pure-unit tests in the same binary still run.
 */
class DbusEnvironment : public ::testing::Environment
{
  public:
    void SetUp() override;
    void TearDown() override;
};

// Base fixture for tests that need the shared D-Bus session.
class DbusMockTestBase : public ::testing::Test
{
  protected:
    // Valid only while hasBus().
    static std::shared_ptr<sdbusplus::asio::connection>& bus();
    static sdbusplus::asio::object_server& objects();
    static mctp::MctpRequester& requester();
    static boost::asio::io_context& ioContext();

    static bool hasBus();

    // GTEST_SKIP() only returns from this function; derived overrides must
    // check testing::Test::IsSkipped() after delegating here.
    void SetUp() override;

    // Drains pending D-Bus work so the next test starts clean.
    void TearDown() override;

    // Run already-queued handlers so destructor-cancelled async ops finish.
    static void drainPendingAsync();

    /**
     * Read a property from this connection's own object server. Async +
     * io_context pump because sd-bus rejects self-directed synchronous calls
     * with ELOOP. Throws SdBusError on error or timeout.
     */
    template <typename PropertyType>
    static PropertyType getProperty(const std::string& path,
                                    const std::string& iface,
                                    const std::string& prop)
    {
        // Heap-owned handler state: a late reply may fire in a later pump,
        // after this frame has thrown ETIMEDOUT and unwound.
        struct State
        {
            std::optional<boost::system::error_code> ec;
            PropertyType value{};
        };
        auto state = std::make_shared<State>();

        sdbusplus::asio::getProperty<PropertyType>(
            *bus(), bus()->get_unique_name(), path, iface, prop,
            [state](boost::system::error_code ec, PropertyType value) {
                state->value = std::move(value);
                state->ec = ec;
            });

        if (!pumpIoUntil([state] { return state->ec.has_value(); },
                         propertyReadTimeout))
        {
            throw sdbusplus::exception::SdBusError(ETIMEDOUT,
                                                   "getProperty timed out");
        }

        const boost::system::error_code callError =
            state->ec.value_or(boost::system::error_code{});
        if (callError)
        {
            throw sdbusplus::exception::SdBusError(
                callError.value(),
                ("getProperty: " + callError.message()).c_str());
        }
        return std::move(state->value);
    }

  private:
    friend class DbusEnvironment;

    static constexpr std::chrono::seconds propertyReadTimeout{5};

    // Pump the shared io_context until done() holds; false on timeout.
    static bool pumpIoUntil(const std::function<bool()>& done,
                            std::chrono::seconds timeout);

    static bool tryConnect();

    // Fresh instance per environment SetUp, never deleted: a late
    // ~connection() must not touch a destroyed io_uring service, and a
    // recreated environment must never run the previous lifetime's queued
    // raw-`this` handlers.
    static boost::asio::io_context* io;
    static std::shared_ptr<sdbusplus::asio::connection> conn;
    static std::unique_ptr<sdbusplus::asio::object_server> objectServer;
    static std::unique_ptr<mctp::MctpRequester> mctpRequester;
    static bool available;
    static std::string bringUpFailure;
};
