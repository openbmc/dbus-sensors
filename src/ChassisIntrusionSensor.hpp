#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <string>

enum IntrusionSensorType
{
    pch,
    gpio,
    hwmon
};

class ChassisIntrusionSensor :
    public std::enable_shared_from_this<ChassisIntrusionSensor>
{
  public:
    ChassisIntrusionSensor(
        boost::asio::io_context& io,
        std::shared_ptr<sdbusplus::asio::dbus_interface> iface);

    ~ChassisIntrusionSensor();

    void start(IntrusionSensorType type, int busId, int slaveAddr,
               bool gpioInverted);

  private:
    std::shared_ptr<sdbusplus::asio::dbus_interface> mIface;
    std::shared_ptr<sdbusplus::asio::connection> mDbusConn;

    IntrusionSensorType mType{IntrusionSensorType::gpio};

    // intrusion status. 0: not intruded, 1: intruded
    std::string mValue = "unknown";
    std::string mOldValue = "unknown";

    // valid if it is PCH register via i2c
    int mBusId{-1};
    int mSlaveAddr{-1};
    boost::asio::steady_timer mPchPollTimer;

    // valid if it is via GPIO
    bool mGpioInverted{false};
    std::string mPinName = "CHASSIS_INTRUSION";
    gpiod::line mGpioLine;
    boost::asio::posix::stream_descriptor mGpioFd;

    // valid if it is via hwmon
    std::string mHwmonName = "intrusion0_alarm";
    std::string mHwmonPath;
    boost::asio::steady_timer mHwmonPollTimer;

    // common members
    bool mOverridenState = false;
    bool mInternalSet = false;

    bool mInitialized = false;

    void updateValue(const std::string& newValue);
    void i2cReadFromPch();
    void pollSensorStatusByPch();
    void readGpio();
    void pollSensorStatusByGpio();
    void initGpioDeviceFile();
    void readHwmon();
    void pollSensorStatusByHwmon();
    void initHwmonDevicePath();
    int setSensorValue(const std::string& req, std::string& propertyValue);
};
