#include <errno.h>
#include <gpiod.h>

#include <GPIOSensor.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <iostream>

boost::asio::io_context io;

struct ObjIfaces
{
    std::shared_ptr<sdbusplus::asio::dbus_interface> cableIfc;
    std::shared_ptr<sdbusplus::asio::dbus_interface> assetTagIfc;
    std::shared_ptr<sdbusplus::asio::dbus_interface> statusIfc;
};

std::unordered_map<std::string, std::pair<GPIOSensing::Config, ObjIfaces>>
    objIfaces;

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
                if (gpiod_line_request_input(line, GPIOSensing::service) < 0)
                {
                    std::cerr
                        << "Unable to request input line: " << strerror(errno)
                        << std::endl;
                    ;
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

// Polling the gpio state every 10 seconds
void startUpdateLoop()
{
    static boost::asio::steady_timer timer(io);
    timer.expires_after(std::chrono::seconds(10));
    timer.async_wait([](const boost::system::error_code& ec) {
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
        for (auto obj : objIfaces)
        {
            auto config = std::get<0>(obj.second);
            auto value = gpioLineRead(config.pinLabel.c_str());
            auto ifaces = std::get<1>(obj.second);
            ifaces.statusIfc->set_property(
                GPIOSensing::Properties::PropertyFunctional,
                config.isActiveLow ? !value : !!value);
        }
        startUpdateLoop();
    });
}

int main()
{
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name(GPIOSensing::service);
    sdbusplus::asio::object_server objectServer(systemBus);

    GPIOSensing::GPIOSensorConfigListener listener(
        GPIOSensing::Interfaces::EMGPIOSensingIfc);

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
                             const GPIOSensing::Config& config) {
            std::string objPath =
                std::string(GPIOSensing::InventoryObjPath) + config.name;
            // Cable Ifc
            auto cableIfc = objectServer.add_interface(
                objPath, GPIOSensing::Interfaces::CableIfc);
            cableIfc->initialize();
            // AssetTag
            auto assetTagIfc = objectServer.add_interface(
                objPath, GPIOSensing::Interfaces::AssetTagIfc);
            assetTagIfc->register_property(
                GPIOSensing::Properties::PropertyAssetTag, config.assetTag);
            assetTagIfc->initialize();
            // Status
            auto value = gpioLineRead(config.pinLabel.c_str());
            auto statusIfc = objectServer.add_interface(
                objPath, GPIOSensing::Interfaces::OperationalStatusIfc);
            statusIfc->register_property(
                GPIOSensing::Properties::PropertyFunctional,
                config.isActiveLow ? !value : !!value);
            statusIfc->initialize();
            ObjIfaces ifaces = {cableIfc, assetTagIfc, statusIfc};
            objIfaces[objPath] = std::make_pair(config, ifaces);
        });

    listener.onInterfaceRemoved(systemBus.get(), [](std::string_view objPath) {
        objIfaces.erase(std::string(objPath));
    });
    startUpdateLoop();
    io.run();
    return 0;
}
