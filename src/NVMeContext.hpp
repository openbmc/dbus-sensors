#pragma once

#include "NVMeSensor.hpp"

#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>

#include <memory>
#include <stdexcept>

class NVMeContext : public std::enable_shared_from_this<NVMeContext>
{
  public:
    NVMeContext(boost::asio::io_service& io, int rootBus) :
        scanTimer(io), rootBus(rootBus), pollCursor(sensors.end())
    {
        if (rootBus < 0)
        {
            throw std::invalid_argument(
                "Invalid root bus: Bus ID must not be negative");
        }
    }

    virtual ~NVMeContext()
    {
        scanTimer.cancel();
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

    // Post-condition: The sensor list does not contain the provided sensor
    // Post-condition: pollCursor is a valid iterator for the sensor list
    void removeSensor(const std::shared_ptr<NVMeSensor>& sensor)
    {
        // Locate the sensor that we're removing in the sensor list
        auto found = std::find(sensors.begin(), sensors.end(), sensor);

        // If we failed to find the sensor in the list the post-condition is
        // already satisfied
        if (found == sensors.end())
        {
            return;
        }

        // We've found the sensor in the list

        // If we're not actively polling the sensor list, then remove the sensor
        if (pollCursor == sensors.end())
        {
            sensors.erase(found);
            return;
        }

        // We're actively polling the sensor list

        // If we're not polling the specific sensor that has been removed, then
        // remove the sensor
        if (*pollCursor != *found)
        {
            sensors.erase(found);
            return;
        }

        // We're polling the sensor that is being removed

        // Remove the sensor and update the poll cursor so the cursor remains
        // valid
        pollCursor = sensors.erase(found);
    }

    virtual void close()
    {
        scanTimer.cancel();
    }

    virtual void pollNVMeDevices() = 0;

    virtual void readAndProcessNVMeSensor() = 0;

    virtual void processResponse(std::shared_ptr<NVMeSensor>& sensor, void* msg,
                                 size_t len) = 0;

  protected:
    boost::asio::steady_timer scanTimer;
    int rootBus; // Root bus for this drive
    std::list<std::shared_ptr<NVMeSensor>> sensors;
    std::list<std::shared_ptr<NVMeSensor>>::iterator pollCursor;
};

using NVMEMap = boost::container::flat_map<int, std::shared_ptr<NVMeContext>>;

NVMEMap& getNVMEMap(void);
