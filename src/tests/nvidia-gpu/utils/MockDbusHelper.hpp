/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MockMctpRequester.hpp"

#include <systemd/sd-bus.h>

#include <MctpRequester.hpp>
#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/message.hpp>

#include <array>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include <gtest/gtest.h>

/**
 * Base test fixture with a shared io_context, connection, object_server,
 * and MctpRequester. Spawns a private dbus-daemon once per binary.
 */
class DbusMockTestBase : public ::testing::Test
{
  protected:
    static boost::asio::io_context io;
    static std::shared_ptr<sdbusplus::asio::connection> conn;
    static std::unique_ptr<sdbusplus::asio::object_server> objectServer;
    static std::unique_ptr<mctp::MctpRequester> mctpRequester;
    static bool initialized;

    static void SetUpTestSuite()
    {
        if (initialized)
        {
            return;
        }

        if (!tryConnect())
        {
            return;
        }
        objectServer = std::make_unique<sdbusplus::asio::object_server>(conn);
        mctpRequester = std::make_unique<mctp::MctpRequester>(io);
        initialized = true;
    }

    void SetUp() override
    {
        if (!initialized)
        {
            GTEST_SKIP() << "No D-Bus session available";
        }
    }

    void TearDown() override
    {
        mock_mctp::clearNextResponse();
        mock_mctp::clearHistory();

        // Drain any pending D-Bus operations (interface removals from
        // destructors) so the next test starts with a clean bus.
        if (initialized)
        {
            sd_bus* const bus =
                sdbusplus::bus::details::bus_friend::get_busp(*conn);
            for (int i = 0; i < 200; i++)
            {
                sd_bus_process(bus, nullptr);
            }
        }
    }

    template <typename T>
    static T getProperty(const std::string& path, const std::string& iface,
                         const std::string& prop)
    {
        sd_bus* const bus =
            sdbusplus::bus::details::bus_friend::get_busp(*conn);

        bool done = false;
        bool gotError = false;
        std::string errorMsg;
        T result{};

        // Read a D-Bus property using async call + sd_bus_process loop.
        // Synchronous sd_bus_call cannot be used here because sd_bus detects
        // re-entrancy (ELOOP) when handling a self-directed method call.
        sdbusplus::message_t method =
            conn->new_method_call(conn->get_unique_name().c_str(), path.c_str(),
                                  "org.freedesktop.DBus.Properties", "Get");
        method.append(iface, prop);

        conn->async_send(method, [&](boost::system::error_code ec,
                                     sdbusplus::message_t& reply) {
            if (ec)
            {
                gotError = true;
                errorMsg = ec.message();
                done = true;
                return;
            }
            try
            {
                std::variant<T> value;
                reply.read(value);
                result = std::get<T>(value);
            }
            catch (const std::exception& e)
            {
                gotError = true;
                errorMsg = e.what();
            }
            done = true;
        });

        for (int i = 0; i < 1000 && !done; i++)
        {
            sd_bus_process(bus, nullptr);
        }

        if (!done)
        {
            throw sdbusplus::exception::SdBusError(ETIMEDOUT,
                                                   "Property read timed out");
        }

        if (gotError)
        {
            throw sdbusplus::exception::SdBusError(ENOENT, errorMsg.c_str());
        }

        return result;
    }

  private:
    // Spawn a private dbus-daemon and connect to it
    static bool tryConnect()
    {
        char* const cwd = get_current_dir_name();
        const std::string tmpDir = std::string(cwd) + "/tmp/dbus";
        free(cwd); // NOLINT(cppcoreguidelines-no-malloc)
        const std::string mkdirCmd = "mkdir -p " + tmpDir;
        // NOLINTNEXTLINE(cert-env33-c)
        if (system(mkdirCmd.c_str()) != 0)
        {
            return false;
        }
        const std::string cmd =
            "TMPDIR=" + tmpDir +
            " dbus-daemon --session "
            "--print-address --fork --address=unix:tmpdir=" +
            tmpDir + " 2>/dev/null";
        std::array<char, 512> buffer{};
        // NOLINTNEXTLINE(cert-env33-c)
        FILE* const pipe = popen(cmd.c_str(), "r");
        if (pipe == nullptr)
        {
            return false;
        }
        std::string addr;
        if (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) !=
            nullptr)
        {
            addr = buffer.data();
            if (!addr.empty() && addr.back() == '\n')
            {
                addr.pop_back();
            }
        }
        pclose(pipe);
        if (addr.empty())
        {
            return false;
        }
        // NOLINTNEXTLINE(concurrency-mt-unsafe)
        setenv("DBUS_SESSION_BUS_ADDRESS", addr.c_str(), 1);
        try
        {
            conn = std::make_shared<sdbusplus::asio::connection>(
                io, sdbusplus::bus::new_default_user());
            return true;
        }
        catch (const sdbusplus::exception_t&)
        {
            return false;
        }
    }
};
