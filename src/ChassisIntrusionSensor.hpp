#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <string>

namespace fs = std::filesystem;

class ChassisIntrusionSensor
{
  public:
    explicit ChassisIntrusionSensor(sdbusplus::asio::object_server& objServer);

    virtual ~ChassisIntrusionSensor();

    void start();

  protected:
    virtual int readSensor() = 0;
    virtual void pollSensorStatus() = 0;
    void updateValue(const size_t& value);

  private:
    // intrusion status. 0: not intruded, 1: intruded
    std::string mValue = "unknown";
    std::string mOldValue = "unknown";
    std::shared_ptr<sdbusplus::asio::dbus_interface> mIface;
    sdbusplus::asio::object_server& mObjServer;
    bool mOverridenState = false;
    bool mInternalSet = false;

    int setSensorValue(const std::string& req, std::string& propertyValue);
};

class ChassisIntrusionPchSensor :
    public ChassisIntrusionSensor,
    public std::enable_shared_from_this<ChassisIntrusionPchSensor>
{
  public:
    ChassisIntrusionPchSensor(boost::asio::io_context& io,
                              sdbusplus::asio::object_server& objServer,
                              int busId, int slaveAddr);

    ~ChassisIntrusionPchSensor() override;

  private:
    int mBusFd{-1};
    int mSlaveAddr{-1};
    boost::asio::steady_timer mPollTimer;
    int readSensor() override;
    void pollSensorStatus() override;
};

class ChassisIntrusionGpioSensor :
    public ChassisIntrusionSensor,
    public std::enable_shared_from_this<ChassisIntrusionGpioSensor>
{
  public:
    ChassisIntrusionGpioSensor(boost::asio::io_context& io,
                               sdbusplus::asio::object_server& objServer,
                               bool gpioInverted);

    ~ChassisIntrusionGpioSensor() override;

  private:
    bool mGpioInverted{false};
    std::string mPinName = "CHASSIS_INTRUSION";
    gpiod::line mGpioLine;
    boost::asio::posix::stream_descriptor mGpioFd;
    int readSensor() override;
    void pollSensorStatus() override;
};
