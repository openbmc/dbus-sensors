#include <GPIOPresenceSensor.hpp>

#include <cerrno>

namespace gpio_presence_sensing
{

void GPIOPresence::setMinPollRate(int newRate)
{
    pollRate = std::min(pollRate, newRate);
}

void GPIOPresence::addInputLine(const std::string& lineLabel)
{
    if (gpioLines.find(lineLabel) == gpioLines.end())
    {
        ::gpiod::line line = ::gpiod::find_line(lineLabel);
        line.request({service, ::gpiod::line_request::DIRECTION_INPUT,
                      /*default_val=*/0});
        gpioLines[lineLabel] = line;
    }
}

int GPIOPresence::readLine(const std::string& lineLabel)
{
    if (gpioLines.find(lineLabel) == gpioLines.end())
    {
        addInputLine(lineLabel);
    }
    return gpioLines[lineLabel].get_value();
}

void GPIOPresence::addObj(
    std::unique_ptr<sdbusplus::asio::dbus_interface>& statusIfc,
    const std::string& objPath, const Config& config)
{
    std::cerr << "New objPath added " << objPath << std::endl;
    objIfaces[objPath] = {std::move(statusIfc), config};
}

void GPIOPresence::removeObj(const std::string& objPath)
{
    if (objIfaces.find(objPath) != objIfaces.end())
    {
        std::cerr << "Remove objPath " << objPath << std::endl;
        objIfaces.erase(objPath);
    }
}

bool GPIOPresence::hasObj(const std::string& objPath)
{
    return objIfaces.find(objPath) != objIfaces.end();
}

void GPIOPresence::startUpdateLoop(bool forceUpdate)
{
    auto weakRef = weak_from_this();
    timer.expires_after(std::chrono::seconds(pollRate));
    timer.async_wait(
        [forceUpdate, weakRef](const boost::system::error_code& ec) mutable {
        std::shared_ptr<GPIOPresence> self = weakRef.lock();
        if (ec)
        {
            // we were cancelled
            std::cerr << "loop canceled: " << ec.message() << std::endl;
            return;
        }

        // libgpiod provides api for event wait. However, it doesn't support
        // lines from different chip. Therefore, we use a polling loop here.
        for (auto& obj : self->objIfaces)
        {
            auto& config = obj.second.config;
            int lineValue = 0;
            try
            {
                lineValue = self->readLine(config.gpioLine);
            }
            catch (const std::invalid_argument& e)
            {
                std::cerr << "Failed gpio line read "
                          << std::string(config.gpioLine)
                          << " error is: " << e.what() << std::endl;
                return;
            }
            catch (const std::runtime_error& e)
            {
                std::cerr << "Failed gpio line read "
                          << std::string(config.gpioLine)
                          << " error is: " << e.what() << std::endl;
                continue;
            }
            bool present = static_cast<bool>(lineValue);
            present = config.activeLow ? !present : present;
            // Only change the state if we detected a difference
            // ForceUpdate will force a dbus update despite the fact that no
            // change is detected. Always force dbus update on the first
            // iteration.
            if (present != config.present || forceUpdate)
            {
                std::cout << "Cable " << config.name << " change state to "
                          << (present ? "connected" : "disconnected")
                          << std::endl;
                obj.second.statusIfc->set_property(
                    gpio_presence_sensing::properties::propertyPresent,
                    present);
                config.present = present;
            }
        }
        self->startUpdateLoop(/*forceUpdate=*/false);
    });
}
} // namespace gpio_presence_sensing
