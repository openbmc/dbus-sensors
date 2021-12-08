#pragma once

#include "NVMeSensor.hpp"

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>

#include <memory>

class NVMeContext : public std::enable_shared_from_this<NVMeContext>
{
  public:
    NVMeContext(boost::asio::io_service& io, int rootBus) :
        scanTimer(io), rootBus(rootBus)
    {}

    virtual ~NVMeContext()
    {
        close();
    }

    void addSensor(std::shared_ptr<NVMeSensor> sensor)
    {
        sensors.emplace_back(sensor);
    }

    std::optional<std::shared_ptr<NVMeSensor>>
        getSensorAtPath(const std::string& path)
    {
        for (auto& sensor : sensors)
        {
            if (sensor->configurationPath == path)
            {
                return sensor;
            }
        }

        return std::nullopt;
    }

    void removeSensor(std::shared_ptr<NVMeSensor> sensor)
    {
        sensors.remove(sensor);
    }

    virtual void pollNVMeDevices()
    {}

    virtual void close()
    {
        scanTimer.cancel();
    }

    virtual void readAndProcessNVMeSensor()
    {}

    virtual void processResponse(void* msg, size_t len)
    {
        (void)msg;
        (void)len;
    }

  protected:
    boost::asio::deadline_timer scanTimer;
    int rootBus;                                    // Root bus for this drive
    std::list<std::shared_ptr<NVMeSensor>> sensors; // used as a poll queue
};

using NVMEMap = boost::container::flat_map<int, std::shared_ptr<NVMeContext>>;

NVMEMap& getNVMEMap(void);
