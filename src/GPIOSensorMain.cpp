#include <errno.h>
#include <gpiod.h>

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
int gpioLineRead(const char* label)
{
    auto line = gpiod_line_find(label);
    int value = -1;
    if (line == nullptr)
    {
        std::cerr << "Unable to find line " << label << std::endl;
    }
    else
    {
        auto dir = gpiod_line_direction(line);
        switch (dir)
        {
            case GPIOD_LINE_DIRECTION_INPUT:
                if (gpiod_line_request_input(line, gpiocablesensing::service) < 0)
                {
                    std::cerr
                        << "Unable to request input line: " << strerror(errno)
                        << std::endl;
                }
                else
                {
                    value = gpiod_line_get_value(line);
                    if (value == -1)
                    {
                        std::cerr << "Read line failed " << label << " err is "
                                  << strerror(errno) << std::endl;
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

// set the fault led group assert to true.
void assertOnFaultLedGroup(std::shared_ptr<sdbusplus::asio::connection> bus,
                           const std::unordered_map<std::string, bool>& faultLedAssert) {
  // no-op if no fault led group is provided.
  for (const auto& faultLed : faultLedAssert) {
    auto led = faultLed.first;
    auto functional = faultLed.second;
    std::string objPath = std::string("/xyz/openbmc_project/led/groups/") + led;
    auto method = bus->new_method_call("xyz.openbmc_project.LED.GroupManager", objPath.c_str(),
                         "org.freedesktop.DBus.Properties", "Set");
    method.append("xyz.openbmc_project.Led.Group", "Asserted", std::variant<bool>(!functional));
    bus->call_noreply(method);
  }
}

// Polling the gpio state every 10 seconds.
void startUpdateLoop(std::shared_ptr<sdbusplus::asio::connection> bus,
                     boost::asio::io_context& io, bool force_update)
{
    static boost::asio::steady_timer timer(io);
    // TODO: why does it need 10 second? Does other numbers work?
    timer.expires_after(std::chrono::seconds(10));
    timer.async_wait([&io, force_update, bus](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            // we were cancelled
            std::cerr << "loop canceled" << std::endl;
            return;
        }
        else if (ec)
        {
            std::cerr << "async wait error " << ec << std::endl;
            return;
        }
        // fault led group name -> assert
        std::unordered_map<std::string, bool> faultLedAssert;
        bool need_change = force_update;
        for (auto& obj : objIfaces)
        {
            auto& config = obj.second.config;
            auto value = gpioLineRead(config.gpioLine.c_str());
            auto functional = config.activeLow ? !value : !!value;
            // Only change the state if we detected a difference to save io to
            // dbus
            if (functional != config.functional || force_update) {
              need_change = true;
              std::cerr << "Cable " << config.name << " change state to "
                        << (functional?"connected":"disconnected") << std::endl;
              obj.second.statusIfc->set_property(
                  gpiocablesensing::Properties::PropertyFunctional, functional);
              config.functional = functional;
              for (auto led : config.faultLedGroup) {
                faultLedAssert[led] |= functional;
              }
            }
        }
        if (need_change) {
          try{
            assertOnFaultLedGroup(bus, faultLedAssert);
          } catch (const sdbusplus::exception::exception& e) {
            std::cerr << "Failed setting fault led: "<< e.what() << std::endl;
          }
        }
        startUpdateLoop(bus, io, false);
    });
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name(gpiocablesensing::service);
    sdbusplus::asio::object_server objectServer(systemBus);

    gpiocablesensing::GPIOSensorConfigListener listener(
        gpiocablesensing::interfaces::EMGPIOCableSensingIfc);

    // This container holds the CableType object.
    // For example, if we have a cable of type xyz and a name of abc,
    // we will create
    // --/xyz
    //  |-/xyz/abc
    // The xyz object (the parent obj) is necessary for ipmi discrete sensor
    std::unordered_map<std::string,
                       std::shared_ptr<sdbusplus::asio::dbus_interface>>
        associationIfcs;
    // Wait for config from EM.
    // Once a new config is posted. We will create the obj on dbus.
    listener.onInterfaceAdded(
        systemBus.get(), [&](std::string_view, std::string_view,
                             const gpiocablesensing::Config& config) {
            std::string objPath =
                std::string(gpiocablesensing::InventoryObjPath) + config.name;
            // Cable Ifc
            auto cableIfc = objectServer.add_unique_interface(
                objPath, gpiocablesensing::interfaces::CableIfc);
            cableIfc->initialize();
            // Connector Ifc
            auto connectorIfc = objectServer.add_unique_interface(
                objPath, gpiocablesensing::interfaces::ConnectorIfc);
            connectorIfc->initialize();
            // Status
            auto statusIfc = objectServer.add_unique_interface(
                objPath, gpiocablesensing::interfaces::OperationalStatusIfc);
            statusIfc->register_property(
                gpiocablesensing::Properties::PropertyFunctional, false);
            statusIfc->initialize();
            objIfaces[objPath] = {std::move(cableIfc), std::move(connectorIfc), std::move(statusIfc), config};
        });

    listener.onInterfaceRemoved(systemBus.get(), [](std::string_view objPath) {
        objIfaces.erase(std::string(objPath));
    });
    startUpdateLoop(systemBus, io, true);
    io.run();
    return 0;
}
