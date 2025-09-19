#include "NotifyWatch.hpp"

#include <sys/inotify.h>
#include <unistd.h>

#include <sdbusplus/async.hpp>

#include <array>
#include <cerrno>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>

namespace notify_watch
{

namespace fs = std::filesystem;

NotifyWatch::NotifyWatch(sdbusplus::async::context& ctx, const std::string& dir,
                         Callback_t callback) :
    ctx(ctx), callback(std::move(callback))
{
    std::error_code ec = {};

    fs::path dirPath(dir);
    if (!fs::create_directories(dirPath, ec))
    {
        if (ec)
        {
            throw std::system_error(ec, "Failed to create directory " + dir);
        }
    }

    fd = inotify_init1(IN_NONBLOCK);
    if (-1 == fd)
    {
        throw std::system_error(errno, std::system_category(),
                                "inotify_init1 failed");
    }

    wd = inotify_add_watch(fd, dir.c_str(), IN_CLOSE_WRITE);
    if (-1 == wd)
    {
        close(fd);
        throw std::system_error(errno, std::system_category(),
                                "inotify_add_watch failed");
    }

    fdioInstance = std::make_unique<sdbusplus::async::fdio>(ctx, fd);
    if (!fdioInstance)
    {
        throw std::system_error(errno, std::system_category(),
                                "Failed to create fdio");
    }
}

NotifyWatch::~NotifyWatch()
{
    if (-1 != fd)
    {
        if (-1 != wd)
        {
            inotify_rm_watch(fd, wd);
        }
        close(fd);
    }
}

auto NotifyWatch::readNotifyAsync() -> sdbusplus::async::task<>
{
    if (!fdioInstance)
    {
        co_return;
    }
    co_await fdioInstance->next();

    alignas(inotify_event) std::array<uint8_t, 4096> buffer{};

    auto bytes = read(fd, buffer.data(), buffer.size());
    if (bytes < 0)
    {
        throw std::system_error(errno, std::system_category(),
                                "Failed to read notify event");
    }

    for (auto* iter = buffer.data(); iter < buffer.data() + bytes;)
    {
        // Bypassed clang tidy warning about reinterpret_cast as cast is being
        // performed to avoid copying of buffer data.
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        std::span<inotify_event> event{reinterpret_cast<inotify_event*>(iter),
                                       1};
        if (((event[0].mask & IN_CLOSE_WRITE) != 0U) &&
            ((event[0].mask & IN_ISDIR) == 0U))
        {
            if (callback)
            {
                // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
                std::span<char> name{reinterpret_cast<char*>(event[0].name),
                                     event[0].len};
                co_await callback(std::string(name.begin(), name.end()));
            }
        }
        iter += sizeof(inotify_event) + event[0].len;
    }

    if (!ctx.stop_requested())
    {
        ctx.spawn(readNotifyAsync());
    }
}

} // namespace notify_watch
