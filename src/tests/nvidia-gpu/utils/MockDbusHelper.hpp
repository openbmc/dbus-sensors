/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MockMctpRequester.hpp"

#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <systemd/sd-bus.h>
#include <unistd.h>

#include <MctpRequester.hpp>
#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/message.hpp>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <system_error>

#include <gtest/gtest.h>

/**
 * Base test fixture with a shared io_context, connection, object_server,
 * and MctpRequester. Spawns a private dbus-daemon once per binary.
 */
class DbusMockTestBase : public ::testing::Test
{
  protected:
    // ---- shared fixture state ----

    static boost::asio::io_context io;
    static std::shared_ptr<sdbusplus::asio::connection> conn;
    static std::unique_ptr<sdbusplus::asio::object_server> objectServer;
    static std::unique_ptr<mctp::MctpRequester> mctpRequester;
    static pid_t daemonPid;
    static bool initialized;

    // ---- gtest lifecycle hooks ----

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

    static void TearDownTestSuite()
    {
        // Tear down sdbusplus objects before killing the daemon so their
        // destructors don't fight with a dead bus.
        mctpRequester.reset();
        objectServer.reset();
        conn.reset();

        stopDaemon(daemonPid);

        initialized = false;
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

        if (!initialized)
        {
            return;
        }

        // Drain any pending D-Bus operations (interface removals from
        // destructors) so the next test starts with a clean bus. sd_bus_process
        // returns >0 while it dispatched a message, 0 once the queue is empty.
        sd_bus* const bus =
            sdbusplus::bus::details::bus_friend::get_busp(*conn);
        while (sd_bus_process(bus, nullptr) > 0)
        {}
    }

    // ---- helpers ----

    // Drain any handlers that are ready to run on the shared io_context, then
    // reset it so the next test starts from a clean state. Used by tests that
    // tear down an object whose destructor cancels timers / async ops.
    static void drainPendingAsync()
    {
        io.poll();
        io.restart();
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

        // Pump the bus until the reply arrives or the wall-clock deadline
        // elapses. When nothing is pending, sd_bus_wait blocks for the
        // remaining budget so we never busy-spin.
        constexpr auto propertyReadTimeout = std::chrono::seconds(5);
        const auto deadline =
            std::chrono::steady_clock::now() + propertyReadTimeout;

        while (!done)
        {
            if (sd_bus_process(bus, nullptr) > 0)
            {
                continue;
            }

            const auto remaining = deadline - std::chrono::steady_clock::now();
            if (remaining <= std::chrono::steady_clock::duration::zero())
            {
                break;
            }

            const auto remainingUsec =
                std::chrono::duration_cast<std::chrono::microseconds>(remaining)
                    .count();
            sd_bus_wait(bus, static_cast<uint64_t>(remainingUsec));
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
    // ---- private dbus-daemon bring-up ----

    static bool tryConnect()
    {
        std::error_code ec;

        const std::filesystem::path tmpDir =
            std::filesystem::current_path(ec) / "tmp" / "dbus";
        if (ec)
        {
            return false;
        }

        std::filesystem::create_directories(tmpDir, ec);
        if (ec)
        {
            return false;
        }

        const std::string tmpDirStr = tmpDir.string();
        const std::string cmd =
            "TMPDIR=" + tmpDirStr +
            " dbus-daemon --session --print-address=1 --print-pid=1 --fork"
            " --address=unix:tmpdir=" +
            tmpDirStr + " 2>/dev/null";

        // NOLINTNEXTLINE(cert-env33-c)
        FILE* const pipe = popen(cmd.c_str(), "r");
        if (pipe == nullptr)
        {
            return false;
        }

        std::array<char, 512> buffer{};
        std::string addr;
        pid_t pid = -1;

        // dbus-daemon prints the address and daemon pid on separate lines.
        for (int line = 0; line < 2; ++line)
        {
            if (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) ==
                nullptr)
            {
                break;
            }

            std::string value = buffer.data();
            if (!value.empty() && value.back() == '\n')
            {
                value.pop_back();
            }

            if (value.find('=') != std::string::npos)
            {
                addr = value;
            }
            else
            {
                char* end = nullptr;
                const long parsedPid = std::strtol(value.c_str(), &end, 10);
                if (end != value.c_str() && *end == '\0' && parsedPid > 0)
                {
                    pid = static_cast<pid_t>(parsedPid);
                }
            }
        }

        pclose(pipe);

        if (addr.empty() || pid <= 0)
        {
            stopDaemon(pid);
            return false;
        }

        sd_bus* rawBus = nullptr;
        if (sd_bus_new(&rawBus) < 0)
        {
            stopDaemon(pid);
            return false;
        }
        if (sd_bus_set_address(rawBus, addr.c_str()) < 0 ||
            sd_bus_set_bus_client(rawBus, 1) < 0 || sd_bus_start(rawBus) < 0)
        {
            sd_bus_unref(rawBus);
            stopDaemon(pid);
            return false;
        }

        try
        {
            conn = std::make_shared<sdbusplus::asio::connection>(
                io, sdbusplus::bus_t(rawBus, std::false_type{}));
            daemonPid = pid;
            return true;
        }
        catch (const sdbusplus::exception_t&)
        {
            stopDaemon(pid);
            return false;
        }
    }

    static bool isProcessGone(pid_t pid)
    {
        return kill(pid, 0) < 0 && errno == ESRCH;
    }

    static void stopDaemon(pid_t& pid)
    {
        if (pid <= 0)
        {
            return;
        }

        kill(pid, SIGTERM);
        // Bounded wait so we never hang if the daemon ignores SIGTERM.
        for (int i = 0; i < 50; ++i)
        {
            if (waitpid(pid, nullptr, WNOHANG) == pid || isProcessGone(pid))
            {
                pid = -1;
                return;
            }
            usleep(20 * 1000);
        }

        kill(pid, SIGKILL);
        for (int i = 0; i < 50; ++i)
        {
            if (waitpid(pid, nullptr, WNOHANG) == pid || isProcessGone(pid))
            {
                break;
            }
            usleep(20 * 1000);
        }
        pid = -1;
    }
};
