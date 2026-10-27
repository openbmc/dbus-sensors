#pragma once

#include <sdbusplus/async.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>

namespace notify_watch
{

class NotifyWatch
{
  public:
    using Callback_t = std::function<sdbusplus::async::task<>(std::string)>;

    NotifyWatch() = delete;
    explicit NotifyWatch(sdbusplus::async::context& ctx, const std::string& dir,
                         Callback_t callback);
    ~NotifyWatch();

    /** @brief Asynchronously watch and notify for any changes to dir */
    auto readNotifyAsync() -> sdbusplus::async::task<>;

  private:
    static bool isTransientReadError(int error);
    auto readNotifyEvents(std::span<uint8_t> buffer) const
        -> std::optional<size_t>;
    auto processNotifyEvents(std::span<uint8_t> buffer)
        -> sdbusplus::async::task<>;

    sdbusplus::async::context& ctx;
    Callback_t callback;
    int wd = -1;
    int fd = -1;
    std::unique_ptr<sdbusplus::async::fdio> fdioInstance;
};

} // namespace notify_watch
