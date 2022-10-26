#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <xyz/openbmc_project/Chassis/Intrusion/server.hpp>

#include <memory>
#include <string>

using namespace sdbusplus::xyz::openbmc_project::Chassis::server;
using RearmMode = Intrusion::RearmMode;

enum IntrusionSensorType
{
    pch,
    gpio
};

class ChassisIntrusionSensor
{
  public:
    ChassisIntrusionSensor(
        boost::asio::io_context& io,
        std::shared_ptr<sdbusplus::asio::dbus_interface> iface);

    ~ChassisIntrusionSensor();

    void start(IntrusionSensorType type, RearmMode rearm, int busId,
               int slaveAddr, bool gpioInverted);

  private:
    std::shared_ptr<sdbusplus::asio::dbus_interface> mIface;
    std::shared_ptr<sdbusplus::asio::connection> mDbusConn;

    IntrusionSensorType mType{IntrusionSensorType::gpio};
    RearmMode mRearm = RearmMode::Automatic;

    // intrusion status. 0: not intruded, 1: intruded
    std::string mValue = "unknown";
    bool mRearmFlag = false;

    // valid if it is PCH register via i2c
    int mBusId{-1};
    int mSlaveAddr{-1};
    boost::asio::steady_timer mPollTimer;

    // valid if it is via GPIO
    bool mGpioInverted{false};
    std::string mPinName = "CHASSIS_INTRUSION";
    gpiod::line mGpioLine;
    boost::asio::posix::stream_descriptor mGpioFd;

    // common members
    bool mOverridenState = false;
    bool mInternalSet = false;

    bool mInitialized = false;

    void updateValue(const std::string& newValue);
    static int i2cReadFromPch(int busId, int slaveAddr);
    void pollSensorStatusByPch();
    void readGpio();
    void pollSensorStatusByGpio();
    void initGpioDeviceFile();
    int setSensorValue(const std::string& req, std::string& propertyValue);
};
