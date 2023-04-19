#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <string>

namespace fs = std::filesystem;

enum IntrusionSensorType
{
    pch,
    gpio,
    hwmon
};

class ChassisIntrusionSensor
{
  public:
    ChassisIntrusionSensor(IntrusionSensorType type,
                           sdbusplus::asio::object_server& objServer);

    virtual ~ChassisIntrusionSensor();

    void start(void);
    IntrusionSensorType getType() const
    {
        return mType;
    }

  protected:
    // intrusion status. 0: not intruded, 1: intruded
    std::string mValue = "unknown";

    virtual void readSensor(void) = 0;
    virtual void pollSensorStatus(void) = 0;
    void updateValue(const std::string& newValue);

  private:
    std::string mOldValue = "unknown";
    std::shared_ptr<sdbusplus::asio::dbus_interface> mIface;
    sdbusplus::asio::object_server& mObjServer;

    // common members
    bool mOverridenState = false;
    bool mInternalSet = false;
    IntrusionSensorType mType = IntrusionSensorType::pch;

    int setSensorValue(const std::string& req, std::string& propertyValue);
};

class ChassisIntrusionPchSensor :
    public ChassisIntrusionSensor,
    public std::enable_shared_from_this<ChassisIntrusionPchSensor>
{
  public:
    ChassisIntrusionPchSensor(IntrusionSensorType type,
                              boost::asio::io_context& io,
                              sdbusplus::asio::object_server& objServer,
                              int busId, int slaveAddr);

    ~ChassisIntrusionPchSensor() override;
    int getBusId() const
    {
        return mBusId;
    }
    int getSlaveAddr() const
    {
        return mSlaveAddr;
    }

  private:
    int mBusFd{-1};
    int mBusId{-1};
    int mSlaveAddr{-1};
    boost::asio::steady_timer mPollTimer;
    void readSensor(void) override;
    void pollSensorStatus(void) override;
};

class ChassisIntrusionGpioSensor :
    public ChassisIntrusionSensor,
    public std::enable_shared_from_this<ChassisIntrusionGpioSensor>
{
  public:
    ChassisIntrusionGpioSensor(IntrusionSensorType type,
                               boost::asio::io_context& io,
                               sdbusplus::asio::object_server& objServer,
                               bool gpioInverted);

    ~ChassisIntrusionGpioSensor() override;

    bool getGpioInverted() const
    {
        return mGpioInverted;
    }

  private:
    bool mGpioInverted{false};
    std::string mPinName = "CHASSIS_INTRUSION";
    gpiod::line mGpioLine;
    boost::asio::posix::stream_descriptor mGpioFd;
    void readSensor(void) override;
    void pollSensorStatus(void) override;
};

class ChassisIntrusionHwmonSensor :
    public ChassisIntrusionSensor,
    public std::enable_shared_from_this<ChassisIntrusionHwmonSensor>
{
  public:
    ChassisIntrusionHwmonSensor(IntrusionSensorType type,
                                boost::asio::io_context& io,
                                sdbusplus::asio::object_server& objServer);

    ~ChassisIntrusionHwmonSensor() override;

  private:
    std::string mHwmonName = "intrusion0_alarm";
    std::string mHwmonPath;
    boost::asio::steady_timer mPollTimer;
    void readSensor(void) override;
    void pollSensorStatus(void) override;
};
