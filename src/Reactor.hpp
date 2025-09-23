#pragma once

#include "Utils.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

class Reactor
{
  public:
    Reactor(const std::string& busNameIn, bool hasManufacturingModeMatchIn,
            boost::asio::io_context& io) :
        busName(busNameIn),
        hasManufacturingModeMatch(hasManufacturingModeMatchIn), io(io),
        systemBus(std::make_shared<sdbusplus::asio::connection>(io)),
        objectServer(systemBus, true),
        sensorsChanged(
            std::make_shared<boost::container::flat_set<std::string>>()),
        filterTimer(io)
    {}

    void requestName()
    {
        systemBus->request_name(busName.c_str());
    };

    int run()
    {
        if (hasManufacturingModeMatch)
        {
            setupManufacturingModeMatch(*systemBus);
        }
        io.run();

        return 0;
    };

    void post(std::function<void()> callback)
    {
        boost::asio::post(io, callback);
    };

  private:
    const std::string busName;
    const bool hasManufacturingModeMatch;

  public:
    boost::asio::io_context& io;
    std::shared_ptr<sdbusplus::asio::connection> systemBus;
    sdbusplus::asio::object_server objectServer;

    // commonly used container to store matches.
    // TODO(alexander): refactor to store DBus matches
    // based on their purpose instead of dumping them all in here.
    // Different purposes may include:
    // - interfaces added
    // - interfaces removed
    // - properties changed
    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches;

    // At the time of writing, 5 reactors had this defined in their main
    // function. Better to define this here once.
    std::shared_ptr<boost::container::flat_set<std::string>> sensorsChanged;

    // commonly used timer, used to buffer reconfiguration change handling
    boost::asio::steady_timer filterTimer;

    // commonly used as callback for properties changed match
    std::function<void(sdbusplus::message_t&)> eventHandler;
};
