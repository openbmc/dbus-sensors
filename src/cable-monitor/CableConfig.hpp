#pragma once

#include <sdbusplus/async.hpp>

#include <string>
#include <vector>

namespace phosphor::cable::config
{

class Config
{
  public:
    Config() = delete;
    explicit Config(sdbusplus::async::context& ctx) : ctx(ctx)
    {
        setupConfigNotify();
    }

    ~Config()
    {
        stopConfigNotify();
    }

    using Cables = std::set<std::string>;

    /** @brief Get the configuration for the cables which are expected to be
     * connected
     */
    auto getExpectedCableConfig() -> sdbusplus::async::task<Cables>;

  private:
    /** @brief Setup notify for configuration changes  */
    auto setupConfigNotify() -> bool;
    /** @brief Stop notify for configuration changes  */
    auto stopConfigNotify() const -> void;
    /** Process the configuration file */
    static auto
        processConfig(std::string configFile) -> sdbusplus::async::task<Cables>;

    /** @brief D-Bus context */
    sdbusplus::async::context& ctx;
    /** @brief Watch descriptor for confile file */
    int wd = -1;
    /** @brief Notify file descriptor */
    int fd = -1;
    /** File descriptor based async event handler */
    std::unique_ptr<sdbusplus::async::fdio> fdioInstance;
};

} // namespace phosphor::cable::config
