#pragma once

#include "Thresholds.hpp"
#include "sensor.hpp"

#include <boost/asio/random_access_file.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

class PSensor
{
  public:
    // Used to map two rotors to ONE GPIOLine.
    static std::unordered_map<std::string, gpiod::line> staticGpioMap;

    virtual bool getValue(void) = 0;
    virtual void monitorPresence(void) = 0;
    virtual ~PSensor() = default;
};

class PresenceSensor : public PSensor
{
  public:
    PresenceSensor(const std::string& gpioName, bool inverted,
                   boost::asio::io_context& io, const std::string& name);
    ~PresenceSensor() override;

    void monitorPresence(void) override;
    void read(void);
    bool getValue(void) override;

  private:
    bool status = true;
    gpiod::line gpioLine;
    boost::asio::posix::stream_descriptor gpioFd;
    std::string name;
};

class PollingPresenceSensor :
    public PSensor,
    public std::enable_shared_from_this<PollingPresenceSensor>
{
  public:
    PollingPresenceSensor(const std::string& gpioName, bool inverted,
                          boost::asio::io_context& io, const std::string& name);
    ~PollingPresenceSensor() override;

    void monitorPresence(void) override;
    bool getValue(void) override;
    void initGpio(const std::string& gpioName, bool inverted);

  private:
    bool fanInserted = true;
    gpiod::line gpioLine;
    std::string name;
    std::string gpioName;
    boost::asio::steady_timer repeatTimer;
};

inline void logFanInserted(const std::string& device)
{
    const auto* msg = "OpenBMC.0.1.FanInserted";
    lg2::error("Fan Inserted", "REDFISH_MESSAGE_ID", msg,
               "REDFISH_MESSAGE_ARGS", device);
}

inline void logFanRemoved(const std::string& device)
{
    const auto* msg = "OpenBMC.0.1.FanRemoved";
    lg2::error("Fan Removed", "REDFISH_MESSAGE_ID", msg, "REDFISH_MESSAGE_ARGS",
               device);
}

inline void logFanRedundancyLost(void)
{
    const auto* msg = "OpenBMC.0.1.FanRedundancyLost";
    lg2::error("Fan Inserted", "REDFISH_MESSAGE_ID", msg);
}

inline void logFanRedundancyRestored(void)
{
    const auto* msg = "OpenBMC.0.1.FanRedundancyRegained";
    lg2::error("Fan Removed", "REDFISH_MESSAGE_ID", msg);
}
