#pragma once

#include "sensor.hpp"

#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>

class PresenceSensor
{
  public:
    PresenceSensor(){};
    PresenceSensor(const std::string& type, const std::string& name) :
        sensorType(type), sensorName(name){};
    virtual ~PresenceSensor() = 0;

    virtual void monitorPresence(void) = 0;
    bool isPresent(void) const
    {
        return status;
    }

  protected:
    gpiod::line gpioLine;
    bool status = false;
    std::string sensorType;
    std::string sensorName;

    inline void logPresent(const std::string& device)
    {
        std::string summary = sensorType + " Present (" + sensorName + ")";
        std::string msg = "OpenBMC.0.1." + sensorType + "Present";
        lg2::error(summary.c_str(), "REDFISH_MESSAGE_ID", msg.c_str(),
                   "REDFISH_MESSAGE_ARGS", device);
    }

    inline void logRemoved(const std::string& device)
    {
        std::string summary = sensorType + " Not Present (" + sensorName + ")";
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
                        const std::string& iSensorName,
                        const std::string& gpioName, bool inverted,
                        boost::asio::io_context& io);

    ~EventPresenceSensor()
    {
        gpioFd.close();
        gpioLine.release();
    }
    void monitorPresence(void) override;
    void read(void);
    void updateAndTracePresence(void);

  private:
    boost::asio::posix::stream_descriptor gpioFd;
};

class sharedGpio
{
    struct gpioUsers
    {
        gpiod::line line;
        unsigned int userCount;
    };
    std::unordered_map<std::string, gpioUsers> gpioMap;

  public:
    void addGpio(const std::string& gpioName, gpiod::line& gpioLine);
    gpiod::line findGpio(const std::string& gpioName);
    void removeGpio(const std::string& gpioName);
};

class PollingPresenceSensor :
    public PresenceSensor,
    public std::enable_shared_from_this<PollingPresenceSensor>
{
    // Used to map multiple objects to a single GPIO line
    static sharedGpio staticGpioMap;

  public:
    PollingPresenceSensor(const std::string& iSensorType,
                          const std::string& iSensorName,
                          const std::string& gpioName, bool inverted,
                          boost::asio::io_context& io);
    ~PollingPresenceSensor()
    {
        staticGpioMap.removeGpio(gpioName);
    }
    void monitorPresence(void) override;
    void initGpio(const std::string& gpioName, bool inverted);
    static inline void afterPollTimerExpires(
        const std::weak_ptr<PollingPresenceSensor>& weakRef,
        const boost::system::error_code& ec);

  private:
    std::string gpioName;
    boost::asio::steady_timer pollTimer;
};
