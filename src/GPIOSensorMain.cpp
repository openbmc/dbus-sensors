#include <errno.h>
#include <gpiod.h>

#include <boost/container/flat_map.hpp>
#include <GPIOSensor.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <iostream>

struct ObjIfaces
{
    std::unique_ptr<sdbusplus::asio::dbus_interface> cableIfc;
    std::unique_ptr<sdbusplus::asio::dbus_interface> ConnectorIfc;
    std::unique_ptr<sdbusplus::asio::dbus_interface> statusIfc;
    gpiocablesensing::Config config;
};

std::unordered_map<std::string, ObjIfaces> objIfaces;

// Helper function read a gpio line
// @param[in] label: gpio line label
int gpioLineRead(const char* label)
{
    auto line = gpiod_line_find(label);
    int value = -1;
    if (line == nullptr)
    {
      std::string err = std::string("Unable to fine line: ") + label;
      throw std::invalid_argument(err);
    }
    else
    {
        auto dir = gpiod_line_direction(line);
        switch (dir)
        {
            case GPIOD_LINE_DIRECTION_INPUT:
                if (gpiod_line_request_input(line, gpiocablesensing::service) <
                    0)
                {
                  std::string err = std::string(
                      "Unable to request input line: ") + strerror(errno);
                  throw std::runtime_error(err);
                }
                else
                {
                    value = gpiod_line_get_value(line);
                    if (value == -1)
                    {
                      std::string err = std::string("Read ") + label
                          + " line failed: " + strerror(errno);
                      throw std::runtime_error(err);
                    }
                }
                break;
            case GPIOD_LINE_DIRECTION_OUTPUT:
                break;
            default:
                break;
        }
        gpiod_line_release(line);
        gpiod_line_close_chip(line);
    }
    return value;
}

// @param[in] faultLedAssert: indicator for if a fault led need to be asserted
void driveFaultLedGroup(
    std::shared_ptr<sdbusplus::asio::connection> bus,
    const boost::container::flat_map<std::string, bool>& faultLedAssert)
{
    // no-op if no fault led group is provided.
    for (const auto& faultLed : faultLedAssert)
    {
        auto led = faultLed.first;
        auto functional = faultLed.second;
        std::string objPath =
            std::string("/xyz/openbmc_project/led/groups/") + led;
        auto method = bus->new_method_call(
            "xyz.openbmc_project.LED.GroupManager", objPath.c_str(),
            "org.freedesktop.DBus.Properties", "Set");
        method.append("xyz.openbmc_project.Led.Group", "Asserted",
                      std::variant<bool>(!functional));
        bus->call_noreply(method);
    }
}

// Polling the gpio state every 1 seconds.
void startUpdateLoop(std::shared_ptr<sdbusplus::asio::connection> bus,
                     bool forceUpdate)
{
    static boost::asio::steady_timer timer(bus->get_io_context());
    timer.expires_after(std::chrono::seconds(1));
    timer.async_wait(
        [forceUpdate, bus](const boost::system::error_code& ec) mutable {
            if (ec)
            {
                // we were cancelled
                std::cerr << "loop canceled: " << ec.message() << std::endl;
                return;
            }
            // fault led group name -> assert
            boost::container::flat_map<std::string, bool> faultLedAssert;
            for (auto& obj : objIfaces)
            {
                auto& config = obj.second.config;
                int lineValue;
                try {
                  lineValue = gpioLineRead(config.gpioLine.c_str());
                } catch (const std::invalid_argument& e) {
                  std::cerr << e.what() << std::endl;
                  return;
                } catch (const std::runtime_error& e) {
                  std::cerr << e.what() << " retry" << std::endl;
                  return startUpdateLoop(bus, false);
                }
                bool functional = config.activeLow ? !lineValue : lineValue;
                // Only change the state if we detected a difference
                if (functional != config.functional || forceUpdate)
                {
                    forceUpdate = true;
                    std::cout << "Cable " << config.name << " change state to "
                              << (functional ? "connected" : "disconnected")
                              << std::endl;
                    obj.second.statusIfc->set_property(
                        gpiocablesensing::Properties::propertyFunctional,
                        functional);
                    config.functional = functional;
                    for (const std::string& ledGroupName : config.faultLedGroup)
                    {
                        faultLedAssert[ledGroupName] |= functional;
                    }
                }
                if (forceUpdate) {
                  try
                  {
                      driveFaultLedGroup(bus, faultLedAssert);
                  }
                  catch (const sdbusplus::exception::exception& e)
                  {
                      std::cerr << "Failed setting fault led: " << e.what()
                                << std::endl;
                  }
                }
            }
            startUpdateLoop(bus, /*forceUpdate=*/ false);
        });
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name(gpiocablesensing::service);
    sdbusplus::asio::object_server objectServer(systemBus);

    gpiocablesensing::GPIOSensorConfigListener listener(
        gpiocablesensing::interfaces::emGPIOCableSensingIfc);
    // Wait for config from EM.
    // Once a new config is posted. We will create the obj on dbus.
    listener.onInterfaceAdded(
        systemBus.get(), [&](std::string_view, std::string_view,
                             const gpiocablesensing::Config& config) {
            std::string objPath =
                std::string(gpiocablesensing::InventoryObjPath) + config.name;
            // Cable Ifc
            auto cableIfc = objectServer.add_unique_interface(
                objPath, gpiocablesensing::interfaces::cableIfc);
            cableIfc->initialize();
            // Connector Ifc
            auto connectorIfc = objectServer.add_unique_interface(
                objPath, gpiocablesensing::interfaces::connectorIfc);
            connectorIfc->initialize();
            // Status
            auto statusIfc = objectServer.add_unique_interface(
                objPath, gpiocablesensing::interfaces::operationalStatusIfc);
            statusIfc->register_property(
                gpiocablesensing::Properties::propertyFunctional, false);
            statusIfc->initialize();
            objIfaces[objPath] = {std::move(cableIfc), std::move(connectorIfc),
                                  std::move(statusIfc), config};
        });

    listener.onInterfaceRemoved(systemBus.get(), [](std::string_view objPath) {
        objIfaces.erase(std::string(objPath));
    });
    startUpdateLoop(systemBus, /*forceUpdate=*/true);
    io.run();
    return 0;
}
