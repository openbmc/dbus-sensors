#include <unistd.h>

#include <BatteryStatus.hpp>

#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

BatteryStatus::BatteryStatus(
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& sensorName,
    const std::string& deviceName, const std::string& sensorConfiguration) :
    Discrete(escapeName(sensorName), sensorConfiguration, conn),
    objServer(objectServer), waitTimer(io), deviceName(deviceName), conn(conn)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/battery/" + name,
        "xyz.openbmc_project.Sensor.State");

    if (!sensorInterface)
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

    setInitialProperties();

    if (!sensorInterface->initialize() || !association->initialize())
    {
        std::cerr << "Error: Failed to initialize DBus interfaces\n";
        return;
    }
}

BatteryStatus::~BatteryStatus()
{
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void BatteryStatus::setupRead(void)
{
    monitorState();
}

void BatteryStatus::monitorState()
{
    int8_t reading = monitorThreshold(deviceName, sensorObjectPath);
    uint16_t state = 0;
    if (reading != -1)
    {
        if (reading & (1 << static_cast<int8_t>(IPMIThresholds::warningLow)))
        {
            state = state | (1 << static_cast<uint16_t>(battery::batteryLow));
        }
        if (reading & (1 << static_cast<int8_t>(IPMIThresholds::criticalLow)))
        {
            state = state |
                    (1 << static_cast<uint16_t>(battery::batteryFailed));
        }
        state = state | (1 << static_cast<uint16_t>(battery::batteryPresence));
    }

    updateState(sensorInterface, state);
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
