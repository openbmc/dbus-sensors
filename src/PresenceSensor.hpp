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

    bool isPresent(void) const
    {
        return status;
    }

  protected:
    gpiod::line gpioLine;
    bool status = false;
    std::string sensorType;
    std::string sensorName;

    virtual void monitorPresence(void) = 0;

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

    void updateAndTracePresence(void);
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

  private:
    boost::asio::posix::stream_descriptor gpioFd;

    void monitorPresence(void) override;
    void read(void);
    void updateAndTracePresence(void);
};
