#pragma once

#include "sensor.hpp"

#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>

class PresenceSensor
{
  public:
    PresenceSensor(const std::string& type, const std::string& name) :
        sensorType(type), sensorName(name) {};
    PresenceSensor(const PresenceSensor&) = delete;
    PresenceSensor& operator=(const PresenceSensor&) = delete;
    virtual ~PresenceSensor() = 0;

    bool isPresent() const
    {
        return status;
    }

  protected:
    gpiod::line gpioLine;
    bool status = false;
    std::string sensorType;
    std::string sensorName;

    virtual void monitorPresence() = 0;

    void logPresent(const std::string& device)
    {
        std::string summary = sensorType + " " + sensorName + " Inserted";
        std::string msg = "OpenBMC.0.1." + sensorType + "Inserted";
        lg2::info(summary.c_str(), "REDFISH_MESSAGE_ID", msg.c_str(),
                  "REDFISH_MESSAGE_ARGS", device);
    }

    void logRemoved(const std::string& device)
    {
        std::string summary = sensorType + " " + sensorName + " Removed";
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

  private:
    boost::asio::posix::stream_descriptor gpioFd;

    void monitorPresence() override;
    void read();
};
