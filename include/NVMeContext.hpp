#pragma once

#include "NVMeSensor.hpp"

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>

#include <memory>
#include <stdexcept>

class NVMeContext : public std::enable_shared_from_this<NVMeContext>
{
  public:
    NVMeContext(boost::asio::io_service& io, int rootBus) :
        scanTimer(io), rootBus(rootBus)
    {
        if (rootBus < 0)
        {
            throw std::invalid_argument(
                "Invalid root bus: Bus ID must not be negative");
        }
    }

    virtual ~NVMeContext()
    {
        close();
    }

    void addSensor(const std::shared_ptr<NVMeSensor>& sensor)
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

    void removeSensor(const std::shared_ptr<NVMeSensor>& sensor)
    {
        sensors.remove(sensor);
    }

    virtual void close()
    {
        scanTimer.cancel();
    }

    virtual void pollNVMeDevices() = 0;

    virtual void readAndProcessNVMeSensor(
        std::list<std::shared_ptr<NVMeSensor>>::iterator iter) = 0;

    virtual void processResponse(std::shared_ptr<NVMeSensor>& sensor, void* msg,
                                 size_t len) = 0;

  protected:
    boost::asio::deadline_timer scanTimer;
    int rootBus;                                    // Root bus for this drive
    std::list<std::shared_ptr<NVMeSensor>> sensors; // used as a poll queue
};

using NVMEMap = boost::container::flat_map<int, std::shared_ptr<NVMeContext>>;

NVMEMap& getNVMEMap(void);
