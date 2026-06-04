/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "DbusMockTestBase.hpp"

#include "MctpRequester.hpp"

#include <linux/prctl.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <systemd/sd-bus.h>
#include <unistd.h>

#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>

// NOLINTNEXTLINE(modernize-deprecated-headers): kill()/SIGKILL are POSIX
#include <signal.h>
// NOLINTNEXTLINE(modernize-deprecated-headers): mkdtemp() is POSIX
#include <stdlib.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <system_error>
#include <thread>
#include <type_traits>

#include <gtest/gtest.h>

[[maybe_unused]] static ::testing::Environment* const dbusEnv =
    ::testing::AddGlobalTestEnvironment(new DbusEnvironment);

boost::asio::io_context* DbusMockTestBase::io = nullptr;
std::shared_ptr<sdbusplus::asio::connection> DbusMockTestBase::conn;
std::unique_ptr<sdbusplus::asio::object_server> DbusMockTestBase::objectServer;
std::unique_ptr<mctp::MctpRequester> DbusMockTestBase::mctpRequester;
bool DbusMockTestBase::available = false;
std::string DbusMockTestBase::bringUpFailure;

namespace
{
pid_t daemonPid = -1;
std::filesystem::path socketDir;

pid_t spawnDaemon(const std::filesystem::path& dir, std::string& addr,
                  std::string& error);
void stopDaemon(pid_t& pid);
void removeSocketDir();
} // namespace

void DbusEnvironment::SetUp()
{
    DbusMockTestBase::io = new boost::asio::io_context;

    if (!DbusMockTestBase::tryConnect())
    {
        return;
    }

    DbusMockTestBase::objectServer =
        std::make_unique<sdbusplus::asio::object_server>(
            DbusMockTestBase::conn);
    DbusMockTestBase::mctpRequester =
        std::make_unique<mctp::MctpRequester>(*DbusMockTestBase::io);

    DbusMockTestBase::available = true;
}

void DbusEnvironment::TearDown()
{
    if (!DbusMockTestBase::available)
    {
        return;
    }

    DbusMockTestBase::io->stop();

    DbusMockTestBase::mctpRequester.reset();
    DbusMockTestBase::objectServer.reset();
    DbusMockTestBase::conn.reset();

    stopDaemon(daemonPid);
    removeSocketDir();

    DbusMockTestBase::available = false;
}

std::shared_ptr<sdbusplus::asio::connection>& DbusMockTestBase::bus()
{
    return conn;
}

sdbusplus::asio::object_server& DbusMockTestBase::objects()
{
    return *objectServer;
}

mctp::MctpRequester& DbusMockTestBase::requester()
{
    return *mctpRequester;
}

boost::asio::io_context& DbusMockTestBase::ioContext()
{
    return *io;
}

bool DbusMockTestBase::hasBus()
{
    return available;
}

void DbusMockTestBase::SetUp()
{
    if (!available)
    {
        GTEST_SKIP() << "No D-Bus session available: " << bringUpFailure;
    }
}

void DbusMockTestBase::TearDown()
{
    if (!available)
    {
        return;
    }

    // Flush and dispatch everything pending so the next test starts clean.
    conn->flush();
    while (conn->process_discard())
    {}
}

void DbusMockTestBase::drainPendingAsync()
{
    if (io->stopped())
    {
        io->restart();
    }
    io->poll();
}

bool DbusMockTestBase::pumpIoUntil(const std::function<bool()>& done,
                                   const std::chrono::seconds timeout)
{
    constexpr auto pumpSlice = std::chrono::milliseconds(50);
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    while (!done())
    {
        if (std::chrono::steady_clock::now() >= deadline)
        {
            return false;
        }
        if (io->stopped())
        {
            io->restart();
        }
        io->run_one_for(pumpSlice);
    }
    return true;
}

bool DbusMockTestBase::tryConnect()
{
    std::string dirTemplate = ::testing::TempDir() + "dbus-mock-XXXXXX";
    if (mkdtemp(dirTemplate.data()) == nullptr)
    {
        bringUpFailure = std::string("mkdtemp failed: ") + strerror(errno);
        return false;
    }
    socketDir = dirTemplate;

    std::string addr;
    daemonPid = spawnDaemon(socketDir, addr, bringUpFailure);
    if (daemonPid < 0)
    {
        removeSocketDir();
        return false;
    }

    if (!addr.starts_with("unix:"))
    {
        bringUpFailure = "dbus-daemon printed no usable address";
        stopDaemon(daemonPid);
        removeSocketDir();
        return false;
    }

    // Connect to the printed address directly; setenv(DBUS_SESSION_BUS_ADDRESS)
    // + bus::new_user() traps under valgrind (hardened glibc setenv).
    // NOLINTNEXTLINE(misc-include-cleaner): sd_bus comes from systemd/sd-bus.h
    sd_bus* rawBus = nullptr;
    int rc = sd_bus_new(&rawBus);
    if (rc >= 0)
    {
        rc = sd_bus_set_address(rawBus, addr.c_str());
    }
    if (rc >= 0)
    {
        rc = sd_bus_set_bus_client(rawBus, 1);
    }
    if (rc >= 0)
    {
        // Must run before bus_t wraps the bus: its constructor calls
        // get_unique_name(), which needs a started bus.
        rc = sd_bus_start(rawBus);
    }
    if (rc < 0)
    {
        bringUpFailure = std::string("bus connect failed: ") + strerror(-rc);
        sd_bus_unref(rawBus);
        stopDaemon(daemonPid);
        removeSocketDir();
        return false;
    }

    try
    {
        // bus_t adopts rawBus and unrefs it on every path.
        conn = std::make_shared<sdbusplus::asio::connection>(
            *io, sdbusplus::bus_t(rawBus, std::false_type{}));
        return true;
    }
    catch (const std::exception& e)
    {
        bringUpFailure = std::string("connection failed: ") + e.what();
        stopDaemon(daemonPid);
        removeSocketDir();
        return false;
    }
}

namespace
{

pid_t spawnDaemon(const std::filesystem::path& dir, std::string& addr,
                  std::string& error)
{
    std::array<int, 2> pipeFds{};
    if (pipe(pipeFds.data()) < 0)
    {
        error = std::string("pipe failed: ") + strerror(errno);
        return -1;
    }

    const pid_t parentPid = getpid();
    const pid_t pid = fork();
    if (pid < 0)
    {
        error = std::string("fork failed: ") + strerror(errno);
        close(pipeFds[0]);
        close(pipeFds[1]);
        return -1;
    }

    if (pid == 0)
    {
        // Die with the test binary even when it crashes past teardown.
        // No --fork: it would reparent the daemon and clear the death signal.
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg): fixed C API
        prctl(PR_SET_PDEATHSIG, SIGTERM);
        // The signal only arms after prctl; bail if the parent already died.
        if (getppid() != parentPid)
        {
            _exit(1);
        }
        dup2(pipeFds[1], STDOUT_FILENO);
        close(pipeFds[0]);
        close(pipeFds[1]);

        // execvp wants mutable argument strings.
        std::string arg0 = "dbus-daemon";
        std::string arg1 = "--session";
        std::string arg2 = "--print-address=1";
        std::string arg3 = "--address=unix:tmpdir=" + dir.string();
        std::array<char*, 5> argv = {arg0.data(), arg1.data(), arg2.data(),
                                     arg3.data(), nullptr};
        execvp(arg0.c_str(), argv.data());
        _exit(127);
    }

    // Read the address line; EOF (empty addr) means execvp failed.
    close(pipeFds[1]);
    FILE* const out = fdopen(pipeFds[0], "r");
    if (out == nullptr)
    {
        error = "fdopen failed";
        close(pipeFds[0]);
        return pid;
    }

    std::array<char, 512> buffer{};
    if (fgets(buffer.data(), static_cast<int>(buffer.size()), out) != nullptr)
    {
        addr = buffer.data();
        if (!addr.empty() && addr.back() == '\n')
        {
            addr.pop_back();
        }
    }
    fclose(out);

    return pid;
}

void stopDaemon(pid_t& pid)
{
    if (pid <= 0)
    {
        return;
    }

    kill(pid, SIGTERM);

    // Direct fork() child, so waitpid is authoritative (no pid-reuse races).
    constexpr auto stopPollInterval = std::chrono::milliseconds(20);
    constexpr auto stopTimeout = std::chrono::seconds(1);
    const auto deadline = std::chrono::steady_clock::now() + stopTimeout;

    while (std::chrono::steady_clock::now() < deadline)
    {
        const pid_t reaped = waitpid(pid, nullptr, WNOHANG);
        if (reaped == pid || (reaped < 0 && errno == ECHILD))
        {
            pid = -1;
            return;
        }
        std::this_thread::sleep_for(stopPollInterval);
    }

    // SIGKILL cannot be ignored; a blocking waitpid reaps promptly.
    kill(pid, SIGKILL);
    waitpid(pid, nullptr, 0);
    pid = -1;
}

void removeSocketDir()
{
    if (socketDir.empty())
    {
        return;
    }
    std::error_code ec;
    std::filesystem::remove_all(socketDir, ec);
    socketDir.clear();
}

} // namespace
