#pragma once

#include <sdbusplus/asio/object_server.hpp>

class ChassisIntrusionSensor
{
  public:
    ChassisIntrusionSensor(
        boost::asio::io_service& io,
        std::shared_ptr<sdbusplus::asio::dbus_interface> iface);

    ~ChassisIntrusionSensor();

    void start(int busId, int slaveAddr);

  private:
    std::shared_ptr<sdbusplus::asio::dbus_interface> mIface;

    // intrusion status. 0: not intruded, 1: intruded
    int mValue;
    int mOldValue;

    // only support PCH register now
    int mBusId;
    int mSlaveAddr;

    int mOverriddenValue = -1;
    bool mOverridenState = false;
    bool mInternalSet = false;

    bool mInitialized = false;

    boost::asio::deadline_timer mPollTimer;

    bool i2cReadFromPch();
    void pollPchInfo();
    int setSensorValue(const int& req, int& propertyValue);
    int updateValue(const int newValue);
};
