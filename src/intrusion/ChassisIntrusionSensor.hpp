#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstddef>
#include <memory>
#include <string>

class ChassisIntrusionSensor :
    public std::enable_shared_from_this<ChassisIntrusionSensor>
{
  public:
    explicit ChassisIntrusionSensor(bool autoRearm, boost::asio::io_context& io,
                                    sdbusplus::asio::object_server& objServer);

    virtual ~ChassisIntrusionSensor();

    virtual void start();

  protected:
    virtual int readSensor() = 0;
    virtual void pollSensorStatus();
    void updateValue(const size_t& value);

  private:
    std::string mValue;
    // If this sensor uses automatic rearm method. Otherwise, manually rearm it
    bool mAutoRearm;
    boost::asio::steady_timer mPollTimer;
    std::shared_ptr<sdbusplus::asio::dbus_interface> mIface;
    sdbusplus::asio::object_server& mObjServer;
    bool mOverridenState = false;
    bool mInternalSet = false;
    bool mRearmFlag = false;

    int setSensorValue(const std::string& req, std::string& propertyValue);
};

class ChassisIntrusionPchSensor : public ChassisIntrusionSensor
{
  public:
    ChassisIntrusionPchSensor(bool autoRearm, boost::asio::io_context& io,
                              sdbusplus::asio::object_server& objServer,
                              int busId, int slaveAddr);

    ~ChassisIntrusionPchSensor() override;

  private:
    int mBusFd{-1};
    int mSlaveAddr{-1};
    int readSensor() override;
};

class ChassisIntrusionGpioSensor : public ChassisIntrusionSensor
{
  public:
    ChassisIntrusionGpioSensor(bool autoRearm, boost::asio::io_context& io,
                               sdbusplus::asio::object_server& objServer,
                               bool gpioInverted);

    ~ChassisIntrusionGpioSensor() override;

    void start() override;

  private:
    bool mGpioInverted{false};
    std::string mPinName = "CHASSIS_INTRUSION";
    gpiod::line mGpioLine;
    boost::asio::posix::stream_descriptor mGpioFd;
    int readSensor() override;
    void pollSensorStatus() override;
};

class ChassisIntrusionHwmonSensor : public ChassisIntrusionSensor
{
  public:
    ChassisIntrusionHwmonSensor(bool autoRearm, boost::asio::io_context& io,
                                sdbusplus::asio::object_server& objServer,
                                std::string hwmonName);

    ~ChassisIntrusionHwmonSensor() override = default;

  private:
    std::string mHwmonName;
    std::string mHwmonPath;
    int readSensor() override;
};
