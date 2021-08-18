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

void GPIOPresence::addObj(
    std::unique_ptr<sdbusplus::asio::dbus_interface> statusIfc,
    std::string_view objPath, const Config& config)
{
    std::cerr << "New objPath added " << objPath << std::endl;
    objIfaces[std::string(objPath)] = {std::move(statusIfc), config};
}

void GPIOPresence::removeObj(std::string_view objPath)
{
    if (objIfaces.find(std::string(objPath)) != objIfaces.end())
    {
        std::cerr << "Remove objPath " << objPath << std::endl;
        objIfaces.erase(std::string(objPath));
    }
}

bool GPIOPresence::hasObj(std::string_view objPath)
{
    return objIfaces.find(std::string(objPath)) != objIfaces.end();
}

void GPIOPresence::startUpdateLoop(bool forceUpdate)
{
    timer.cancel();
    timer.expires_after(std::chrono::seconds(pollRate));
    timer.async_wait([forceUpdate,
                      this](const boost::system::error_code& ec) mutable {
        if (ec)
        {
            // we were cancelled
            std::cerr << "loop canceled: " << ec.message() << std::endl;
            return;
        }

        // libgpiod provides api for event wait. However, it doesn't support
        // lines from different chip. Therefore, we use a polling loop here.
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
            }
        }
        startUpdateLoop(/*forceUpdate=*/false);
    });
}
} // namespace gpiopresencesensing
