#pragma once

#include "Reactor.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

// T should be the class derived from 'Sensor'
template <typename T>
class SensorReactor : public Reactor
{
  public:
    SensorReactor(const std::string& busName, bool hasManufacturingModeMatch,
                  const std::vector<std::string>& managerPaths) :
        Reactor(busName, hasManufacturingModeMatch)
    {
        for (const auto& objPath : managerPaths)
        {
            objectServer.add_manager(objPath);
        }
        systemBus->request_name(busName.c_str());
    }

    SensorReactor(const std::string& busName, bool hasManufacturingModeMatch) :
        SensorReactor(busName, hasManufacturingModeMatch,
                      {"/xyz/openbmc_project/sensors"})
    {}

    boost::container::flat_map<std::string, std::shared_ptr<T>> sensors;
};
