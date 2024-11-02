#pragma once

#include <sys/inotify.h>
#include <unistd.h>

#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/fdio.hpp>
#include <sdbusplus/async/task.hpp>

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <system_error>

namespace phosphor::notify::watch
{

namespace fs = std::filesystem;

template <typename Instance>
class NotifyWatch
{
  public:
    NotifyWatch() = delete;

    explicit NotifyWatch(sdbusplus::async::context& ctx,
                         const std::string& dir) : ctx(ctx)
    {
        std::error_code ec = {};

        fs::path dirPath(dir);
        if (!fs::create_directories(dirPath, ec))
        {
            if (ec)
            {
                throw std::system_error(ec,
                                        "Failed to create directory " + dir);
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
    }

    ~NotifyWatch()
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

    sdbusplus::async::task<> readNotifyAsync()
    {
        co_await fdioInstance->next();

        constexpr size_t maxBytes = 1024;
        std::array<uint8_t, maxBytes> buffer{};

        auto bytes = read(fd, buffer.data(), maxBytes);
        if (0 > bytes)
        {
            throw std::system_error(errno, std::system_category(),
                                    "Failed to read notify event");
        }
        auto offset = 0;
        while (offset < bytes)
        {
            // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
            std::span<uint32_t> mask{
                reinterpret_cast<uint32_t*>(
                    buffer.data() + offset + offsetof(inotify_event, mask)),
                1};
            std::span<uint32_t> len{
                reinterpret_cast<uint32_t*>(
                    buffer.data() + offset + offsetof(inotify_event, len)),
                1};
            // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)

            if (((mask[0] & IN_CLOSE_WRITE) != 0U) &&
                ((mask[0] & IN_ISDIR) == 0U))
            {
                // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
                std::span<char> name{
                    reinterpret_cast<char*>(
                        buffer.data() + offset + offsetof(inotify_event, name)),
                    len[0]};
                // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
                co_await static_cast<Instance*>(this)->processConfigUpdate(
                    std::string(name.begin(), name.end()));
            }
            offset += offsetof(inotify_event, name) + len[0];
        }

        if (!ctx.stop_requested())
        {
            ctx.spawn(readNotifyAsync());
        }
    }

  private:
    sdbusplus::async::context& ctx;
    int wd = -1;
    int fd = -1;
    std::unique_ptr<sdbusplus::async::fdio> fdioInstance;
};

} // namespace phosphor::notify::watch
