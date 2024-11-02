#pragma once

#include <nlohmann/json.hpp>
#include <sdbusplus/async.hpp>

#include <string>
#include <vector>

namespace phosphor::cable::config
{

static constexpr auto configFileDir = "/var/lib/cablemonitor.d";
static constexpr auto configFileName = "cable-config.json";

class Config
{
    using json = nlohmann::json;

  public:
    Config() = delete;
    explicit Config(sdbusplus::async::context& ctx) : ctx(ctx) {}

    using Cables = std::set<std::string>;

    /** Process the configuration file */
    static auto
        processConfig(std::string configFile) -> sdbusplus::async::task<Cables>;

  private:
    /** @brief Parse the configuration file */
    static auto
        parseConfigFile(std::string configFile) -> sdbusplus::async::task<json>;

    sdbusplus::async::context& ctx;
};

} // namespace phosphor::cable::config
