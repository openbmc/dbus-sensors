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

class PresenceSensor
{
  public:
    virtual void monitorPresence(void) = 0;
    virtual ~PresenceSensor()
    {
        gpioLine.release();
    }

    bool isPresent(void) const
    {
        return status;
    }

  protected:
    gpiod::line gpioLine;
    std::string name;
    bool status = true;
    std::string sensorType;

    inline void logInserted(const std::string& device)
    {
        std::string summary = sensorType + " Inserted";
        std::string msg = "OpenBMC.0.1." + sensorType + "Inserted";
        lg2::error(summary.c_str(), "REDFISH_MESSAGE_ID", msg.c_str(),
                   "REDFISH_MESSAGE_ARGS", device);
    }

    inline void logRemoved(const std::string& device)
    {
        std::string summary = sensorType + " Removed";
        std::string msg = "OpenBMC.0.1." + sensorType + "Removed";
        lg2::error(summary.c_str(), "REDFISH_MESSAGE_ID", msg.c_str(),
                   "REDFISH_MESSAGE_ARGS", device);
    }
};

class EventPresenceSensor :
    public PresenceSensor,
    public std::enable_shared_from_this<EventPresenceSensor>
{
  public:
    EventPresenceSensor(const std::string& iSensorType,
                        const std::string& gpioName, bool inverted,
                        boost::asio::io_context& io, const std::string& name);

    ~EventPresenceSensor() override
    {
        gpioFd.close();
    }
    void monitorPresence(void) override;
    void read(void);
    void tracePresence(void);

  private:
    boost::asio::posix::stream_descriptor gpioFd;
};

class PollingPresenceSensor :
    public PresenceSensor,
    public std::enable_shared_from_this<PollingPresenceSensor>
{
    // Used to map two rotors to ONE GPIOLine.
    static std::unordered_map<std::string, gpiod::line> staticGpioMap;

  public:
    PollingPresenceSensor(const std::string& iSensorType,
                          const std::string& gpioName, bool inverted,
                          boost::asio::io_context& io, const std::string& name);

    ~PollingPresenceSensor() override
    {
        staticGpioMap.erase(gpioName);
    }
    void monitorPresence(void) override;
    void initGpio(const std::string& gpioName, bool inverted);

  private:
    std::string gpioName;
    boost::asio::steady_timer pollTimer;
};
