#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>
#include <gpiod.hpp>
#include <phosphor-logging/commit.hpp>
#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/State/Fan/event.hpp>

#include <memory>
#include <string>

class PresenceGpio
{
  public:
    PresenceGpio(const std::string& deviceType, const std::string& deviceName,
                 const std::string& gpioName);
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
        lg2::info(summary.c_str());

        const sdbusplus::object_path fanPath{
            "/xyz/openbmc_project/inventory/" + device};
        lg2::commit(
            sdbusplus::event::xyz::openbmc_project::state::Fan::FanInserted(
                "FAN_NAME", fanPath));
    }

    void logRemoved(const std::string& device)
    {
        std::string summary = deviceType + " " + deviceName + " Removed";
        lg2::error(summary.c_str());

        const sdbusplus::object_path fanPath{
            "/xyz/openbmc_project/inventory/" + device};
        lg2::commit(
            sdbusplus::event::xyz::openbmc_project::state::Fan::FanRemoved(
                "FAN_NAME", fanPath));
    }

    void updateAndTracePresence(int newValue);
};

class EventPresenceGpio :
    public PresenceGpio,
    public std::enable_shared_from_this<EventPresenceGpio>
{
  public:
    EventPresenceGpio(const std::string& deviceType,
                      const std::string& deviceName,
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
    PollingPresenceGpio(const std::string& deviceType,
                        const std::string& deviceName,
                        const std::string& gpioName, bool inverted,
                        boost::asio::io_context& io);
    ~PollingPresenceGpio() override
    {
        // GPIO no longer being used so release/remove
        gpioLine.release();
    }
    void monitorPresence() override;

  private:
    boost::asio::steady_timer pollTimer;

    static inline void pollTimerHandler(
        const std::weak_ptr<PollingPresenceGpio>& weakRef,
        const boost::system::error_code& ec);
};
