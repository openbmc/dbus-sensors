#pragma once

#include "sensor.hpp"

#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>

class PresenceGpio
{
  public:
    PresenceGpio(const std::string& iDeviceType, const std::string& iDeviceName,
                 const std::string& iGpioName);
    PresenceGpio(const PresenceGpio&) = delete;
    PresenceGpio& operator=(const PresenceGpio&) = delete;
    virtual ~PresenceGpio() = 0;

    virtual void monitorPresence() = 0;
    bool isPresent() const
    {
        return status;
    }

  protected:
    gpiod::line gpioLine;
    bool status = false;
    std::string deviceType;
    std::string deviceName;
    std::string gpioName;

    void logPresent(const std::string& device)
    {
        std::string summary = deviceType + " " + deviceName + " Inserted";
        std::string msg = "OpenBMC.0.1." + deviceType + "Inserted";
        lg2::info(summary.c_str(), "REDFISH_MESSAGE_ID", msg.c_str(),
                  "REDFISH_MESSAGE_ARGS", device);
    }

    void logRemoved(const std::string& device)
    {
        std::string summary = deviceType + " " + deviceName + " Removed";
        std::string msg = "OpenBMC.0.1." + deviceType + "Removed";
        lg2::error(summary.c_str(), "REDFISH_MESSAGE_ID", msg.c_str(),
                   "REDFISH_MESSAGE_ARGS", device);
    }

    void updateAndTracePresence(int newValue);
};

class EventPresenceGpio :
    public PresenceGpio,
    public std::enable_shared_from_this<EventPresenceGpio>
{
  public:
    EventPresenceGpio(const std::string& iDeviceType,
                      const std::string& iDeviceName,
                      const std::string& gpioName, bool inverted,
                      boost::asio::io_context& io);

    void monitorPresence() override;

  private:
    boost::asio::posix::stream_descriptor gpioFd;

    void read();
};

class PollingPresenceGpio :
    public PresenceGpio,
    public std::enable_shared_from_this<PollingPresenceGpio>
{
  public:
    PollingPresenceGpio(const std::string& iDeviceType,
                        const std::string& iDeviceName,
                        const std::string& iGpioName, bool inverted,
                        boost::asio::io_context& io);
    ~PollingPresenceGpio() override
    {
        // GPIO no longer being used so release/remove
        gpioLine.release();
    }
    void monitorPresence() override;

  private:
    boost::asio::steady_timer pollTimer;

    static inline void
        pollTimerHandler(const std::weak_ptr<PollingPresenceGpio>& weakRef,
                         const boost::system::error_code& ec);
};
