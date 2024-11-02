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
        std::error_code ec;

        fs::path dirPath(dir);
        if (!fs::is_directory(dirPath, ec))
        {
            fs::create_directories(dir, ec);
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

        constexpr auto maxBytes = 1024;
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
            uint32_t mask = 0;
            uint32_t len = 0;
            std::array<char, maxBytes> name{};

            std::memcpy(&mask,
                        buffer.data() + offset + offsetof(inotify_event, mask),
                        sizeof(mask));
            std::memcpy(&len,
                        buffer.data() + offset + offsetof(inotify_event, len),
                        sizeof(len));
            std::memcpy(name.data(),
                        buffer.data() + offset + offsetof(inotify_event, name),
                        len);
            name[len] = '\0';

            if (((mask & IN_CLOSE_WRITE) != 0U) && ((mask & IN_ISDIR) == 0U))
            {
                co_await static_cast<Instance*>(this)->processConfigUpdate(
                    name.data());
            }

            offset += offsetof(inotify_event, name) + len;
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
