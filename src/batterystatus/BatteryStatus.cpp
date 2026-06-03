#include "Discrete.hpp"
#include "Utils.hpp"

#include <BatteryStatus.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

namespace
{
constexpr const char* operationalStatusInterface =
    "xyz.openbmc_project.State.Decorator.OperationalStatus";
constexpr const char* availabilityInterface =
    "xyz.openbmc_project.State.Decorator.Availability";
constexpr int8_t criticalLowMask = 0x04;
} // namespace

BatteryStatus::BatteryStatus(
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& sensorName,
    const std::string& deviceName, const std::string& sensorConfiguration) :
    Discrete(escapeName(sensorName), sensorConfiguration, conn),
    objServer(objectServer), waitTimer(io), deviceName(deviceName)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/battery/" + name,
        operationalStatusInterface);

    if (!sensorInterface)
    {
        std::cerr << "Error: Failed to create DBus interfaces\n";
        return;
    }

    availableInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/battery/" + name, availabilityInterface);

    if (!availableInterface)
    {
        std::cerr << "Error: Failed to create DBus interfaces\n";
        return;
    }

    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/battery/" + name, association::interface);

    if (!association)
    {
        std::cerr << "Error: Failed to create DBus interfaces\n";
        return;
    }

    createAssociation(association, configurationPath);
    sensorInterface->register_property("Functional", true);
    availableInterface->register_property("Available", true);

    if (!sensorInterface->initialize() || !availableInterface->initialize() ||
        !association->initialize())
    {
        std::cerr << "Error: Failed to initialize DBus interfaces\n";
        return;
    }
}

BatteryStatus::~BatteryStatus()
{
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(availableInterface);
    objServer.remove_interface(association);
}

void BatteryStatus::setupRead()
{
    monitorState();
}

void BatteryStatus::monitorState()
{
    int8_t thresholdAlarms = monitorThreshold(deviceName, sensorObjectPath);
    bool available = (thresholdAlarms != -1);
    bool functional = available;

    if (available && ((thresholdAlarms & criticalLowMask) != 0))
    {
        functional = false;
    }

    if (availableInterface)
    {
        availableInterface->set_property("Available", available);
    }

    if (sensorInterface)
    {
        sensorInterface->set_property("Functional", functional);
    }

    restartRead();
}

void BatteryStatus::restartRead()
{
    std::weak_ptr<BatteryStatus> weakRef = weak_from_this();
    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        if (ec)
        {
            std::cerr << "error in restartRead\n" << std::endl;
            return;
        }
        std::shared_ptr<BatteryStatus> self = weakRef.lock();
        if (!self)
        {
            return;
        }
        self->setupRead();
    });
}
