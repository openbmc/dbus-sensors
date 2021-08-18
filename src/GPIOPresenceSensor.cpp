#include <GPIOPresenceSensor.hpp>

#include <cerrno>

namespace gpiopresencesensing
{

void GPIOPresence::addInputLine(std::string_view line_label)
{
    if (gpioLines.find(std::string(line_label)) == gpioLines.end())
    {
        ::gpiod::line line = ::gpiod::find_line(std::string(line_label));
        line.request({service, ::gpiod::line_request::DIRECTION_INPUT, 0});
        gpioLines[std::string(line_label)] = line;
    }
}

int GPIOPresence::readLine(std::string_view line_label)
{
    if (gpioLines.find(std::string(line_label)) == gpioLines.end())
    {
        addInputLine(line_label);
    }
    return gpioLines[std::string(line_label)].get_value();
}

void GPIOPresence::releaseLines()
{
    for (const auto& line : gpioLines)
    {
        line.second.release();
    }
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
    std::string_view objPath, const Config& config)
{
    objIfaces[std::string(objPath)] = {std::move(inventoryIfc),
                                       std::move(statusIfc), config};
}

void GPIOPresence::removeObj(std::string_view objPath)
{
    objIfaces.erase(std::string(objPath));
}

bool GPIOPresence::hasObj(std::string_view objPath)
{
    return objIfaces.find(std::string(objPath)) != objIfaces.end();
}

// Polling the gpio state every 1 seconds.
void GPIOPresence::startUpdateLoop(bool forceUpdate)
{
    static boost::asio::steady_timer timer(bus->get_io_context());
    timer.expires_after(std::chrono::seconds(pollRate));
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
                lineValue = readLine(config.gpioLine);
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
