#pragma once

#include "sensor.hpp"

#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>

class PresenceSensor : private boost::noncopyable
{
  public:
    PresenceSensor(const std::string& type, const std::string& name) :
        sensorType(type), sensorName(name){};
    virtual ~PresenceSensor() = 0;

    virtual void monitorPresence() = 0;
    bool isPresent() const
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
        lg2::info(summary.c_str(), "REDFISH_MESSAGE_ID", msg.c_str(),
                  "REDFISH_MESSAGE_ARGS", device);
    }

    inline void logRemoved(const std::string& device)
    {
        std::string summary = sensorType + " Not Present (" + sensorName + ")";
        std::string msg = "OpenBMC.0.1." + sensorType + "Removed";
        lg2::error(summary.c_str(), "REDFISH_MESSAGE_ID", msg.c_str(),
                   "REDFISH_MESSAGE_ARGS", device);
    }

    void updateAndTracePresence();
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

    ~EventPresenceSensor() override
    {
        gpioFd.close();
    }
    void monitorPresence() override;

  private:
    boost::asio::posix::stream_descriptor gpioFd;

    void read();
};

class PollingPresenceSensor :
    public PresenceSensor,
    public std::enable_shared_from_this<PollingPresenceSensor>
{
  public:
    PollingPresenceSensor(const std::string& iSensorType,
                          const std::string& iSensorName,
                          const std::string& iGpioName, bool inverted,
                          boost::asio::io_context& io);
    ~PollingPresenceSensor() override
    {
        // GPIO no longer being used so release/remove
        gpioLine.release();
    }
    void monitorPresence() override;

  private:
    std::string gpioName;
    boost::asio::steady_timer pollTimer;

    void initGpio(bool inverted);
    static inline void
        pollTimerHandler(const std::weak_ptr<PollingPresenceSensor>& weakRef,
                         const boost::system::error_code& ec);
};
