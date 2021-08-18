#include <gpiod.h>

#include <GPIOPresenceSensor.hpp>

#include <cerrno>

namespace gpiopresencesensing
{

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
    auto dir = gpiod_line_direction(line);
    switch (dir)
    {
        case GPIOD_LINE_DIRECTION_INPUT:
            if (gpiod_line_request_input(line, gpiopresencesensing::service) <
                0)
            {
                std::string err =
                    std::string("Unable to request input line: ") +
                    strerror(errno);
                throw std::runtime_error(err);
            }
            else
            {
                value = gpiod_line_get_value(line);
                if (value == -1)
                {
                    std::string err = std::string("Read ") + label +
                                      " line failed: " + strerror(errno);
                    throw std::runtime_error(err);
                }
            }
            break;
        default:
            break;
    }
    gpiod_line_release(line);
    gpiod_line_close_chip(line);
    return value;
}

void GPIOPresence::driveFaultLedGroup(
    const boost::container::flat_map<std::string, bool>& faultLedAssert)
{
    // no-op if no fault led group is provided.
    for (const auto& faultLed : faultLedAssert)
    {
        auto led = faultLed.first;
        auto assert = faultLed.second;
        std::string objPath =
            std::string("/xyz/openbmc_project/led/groups/") + led;
        auto method = bus->new_method_call(
            "xyz.openbmc_project.LED.GroupManager", objPath.c_str(),
            "org.freedesktop.DBus.Properties", "Set");
        method.append("xyz.openbmc_project.Led.Group", "Asserted",
                      std::variant<bool>(assert));
        bus->call_noreply(method);
    }
}

void GPIOPresence::addObj(
    std::unique_ptr<sdbusplus::asio::dbus_interface> inventoryIfc,
    std::unique_ptr<sdbusplus::asio::dbus_interface> statusIfc,
    const std::string& objPath, const Config& config)
{
    objIfaces[objPath] = {std::move(inventoryIfc), std::move(statusIfc),
                          config};
}

void GPIOPresence::removeObj(const std::string& objPath)
{
    objIfaces.erase(objPath);
}

// Polling the gpio state every 1 seconds.
void GPIOPresence::startUpdateLoop(bool forceUpdate)
{
    static boost::asio::steady_timer timer(bus->get_io_context());
    timer.expires_after(std::chrono::seconds(1));
    timer.async_wait([forceUpdate,
                      this](const boost::system::error_code& ec) mutable {
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
            try
            {
                lineValue = gpioLineRead(config.gpioLine.c_str());
            }
            catch (const std::invalid_argument& e)
            {
                std::cerr << e.what() << std::endl;
                return;
            }
            catch (const std::runtime_error& e)
            {
                std::cerr << e.what() << " retry" << std::endl;
                return startUpdateLoop(false);
            }
            bool present = config.activeLow ? !lineValue : lineValue;
            // Only change the state if we detected a difference
            if (present != config.present || forceUpdate)
            {
                forceUpdate = true;
                std::cout << "Cable " << config.name << " change state to "
                          << (present ? "connected" : "disconnected")
                          << std::endl;
                obj.second.statusIfc->set_property(
                    gpiopresencesensing::Properties::propertyPresent, present);
                config.present = present;
                for (const std::string& ledGroupName : config.faultLedGroup)
                {
                    faultLedAssert[ledGroupName] |= !present;
                }
            }
        }
        if (forceUpdate)
        {
            try
            {
                driveFaultLedGroup(faultLedAssert);
            }
            catch (const sdbusplus::exception::exception& e)
            {
                std::cerr << "Failed setting fault led: " << e.what()
                          << std::endl;
            }
        }
        startUpdateLoop(/*forceUpdate=*/false);
    });
}

const char* classToInterface(std::string_view className)
{
    if (className == "Cable")
    {
        return gpiopresencesensing::interfaces::cableIfc;
    }
    // Keep adding
    return nullptr;
}
} // namespace gpiopresencesensing
