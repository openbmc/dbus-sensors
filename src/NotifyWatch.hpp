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

namespace notify_watch
{

namespace fs = std::filesystem;

class NotifyWatch
{
  public:
    using Callback_t = std::function<sdbusplus::async::task<>(std::string)>;

    NotifyWatch() = delete;
    explicit NotifyWatch(sdbusplus::async::context& ctx, const std::string& dir,
                         Callback_t callback);
    ~NotifyWatch();

    /** @brief Asynchronously watch and notify for any changes to dir */
    sdbusplus::async::task<> readNotifyAsync();

  private:
    sdbusplus::async::context& ctx;
    Callback_t callback;
    int wd = -1;
    int fd = -1;
    std::unique_ptr<sdbusplus::async::fdio> fdioInstance;
};

} // namespace notify_watch
