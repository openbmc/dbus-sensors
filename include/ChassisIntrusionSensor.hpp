#pragma once

#include <sdbusplus/asio/object_server.hpp>

enum IntrusionSensorType
{
    pch,
    gpio
};

class ChassisIntrusionSensor
{
  public:
    ChassisIntrusionSensor(
        boost::asio::io_service& io,
        std::shared_ptr<sdbusplus::asio::dbus_interface> iface);

    ~ChassisIntrusionSensor();

    void start(IntrusionSensorType type, int busId, int slaveAddr,
               int gpioIndex, bool gpioInverted);

  private:
    std::shared_ptr<sdbusplus::asio::dbus_interface> mIface;
    std::shared_ptr<sdbusplus::asio::connection> mDbusConn;

    IntrusionSensorType mType;

    // intrusion status. 0: not intruded, 1: intruded
    std::string mValue = "unknown";
    std::string mOldValue = "unknown";

    // valid if it is PCH register via i2c
    int mBusId;
    int mSlaveAddr;
    boost::asio::deadline_timer mPollTimer;

    // valid if it is via GPIO
    int mGpioIndex;
    bool mGpioInverted;
    boost::asio::ip::tcp::socket mInputDev;
    int mFd;

    // common members
    std::string mOverriddenValue = "unknown";
    bool mOverridenState = false;
    bool mInternalSet = false;

    bool mInitialized = false;

    void updateValue(const std::string newValue);
    int i2cReadFromPch(int busId, int slaveAddr);
    void pollSensorStatusByPch();
    void readGpio();
    void pollSensorStatusByGpio();
    void initGpioDeviceFile(const int index);
    int setSensorValue(const std::string& req, std::string& propertyValue);
};
