#include "IpmbSDRSensor.hpp"
#include "IpmbSensor.hpp"
#include "Utils.hpp"

#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

std::unique_ptr<boost::asio::steady_timer> initCmdTimer;
boost::container::flat_map<std::string, std::shared_ptr<IpmbSensor>> sensors;
boost::container::flat_map<uint8_t, std::shared_ptr<IpmbSDRDevice>> sdrsensor;

void sdrHandler(
    boost::container::flat_map<uint8_t, std::shared_ptr<IpmbSDRDevice>>
        sdrsensor,
    sdbusplus::message_t& message,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    std::string objectName;
    SensorBaseConfigMap values;
    message.read(objectName, values);

    auto findBus = values.find("Bus");
    if (findBus == values.end())
    {
        return;
    }

    uint8_t busIndex = loadVariant<uint8_t>(values, "Bus");

    auto& sdrsen = sdrsensor[busIndex];
    sdrsen = nullptr;
    sdrsen = std::make_shared<IpmbSDRDevice>(dbusConnection, busIndex);
    sdrsen->getSDRRepositoryInfo();
}

void reinitSensors(sdbusplus::message_t& message)
{
    constexpr const size_t reinitWaitSeconds = 2;
    std::string objectName;
    boost::container::flat_map<std::string, std::variant<std::string>> values;
    message.read(objectName, values);

    auto findStatus = values.find(power::property);
    if (findStatus != values.end())
    {
        bool powerStatus =
            std::get<std::string>(findStatus->second).ends_with(".Running");
        if (powerStatus)
        {
            if (!initCmdTimer)
            {
                // this should be impossible
                return;
            }
            // we seem to send this command too fast sometimes, wait before
            // sending
            initCmdTimer->expires_after(
                std::chrono::seconds(reinitWaitSeconds));

            initCmdTimer->async_wait([](const boost::system::error_code ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }

                for (const auto& [name, sensor] : sensors)
                {
                    if (sensor)
                    {
                        sensor->runInitCmd();
                    }
                }
            });
        }
    }
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");
    systemBus->request_name("xyz.openbmc_project.IpmbSensor");

    initCmdTimer = std::make_unique<boost::asio::steady_timer>(io);

    boost::asio::post(io, [&]() {
        createSensors(io, objectServer, sensors, systemBus);
    });

    boost::asio::steady_timer configTimer(io);

    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&](sdbusplus::message_t&) {
            configTimer.expires_after(std::chrono::seconds(1));
            // create a timer because normally multiple properties change
            configTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }
                createSensors(io, objectServer, sensors, systemBus);
                if (sensors.empty())
                {
                    lg2::info("Configuration not detected");
                }
            });
        };

    static constexpr std::array<std::string_view, 1> sensorTypes{{sensorType}};

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(*systemBus, sensorTypes, eventHandler);

    sdbusplus::bus::match_t powerChangeMatch(
        static_cast<sdbusplus::bus_t&>(*systemBus),
        "type='signal',interface='" + std::string(properties::interface) +
            "',path='" + std::string(power::path) + "',arg0='" +
            std::string(power::interface) + "'",
        reinitSensors);

    auto matchSignal = std::make_shared<sdbusplus::bus::match_t>(
        static_cast<sdbusplus::bus_t&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" +
            configInterfaceName(sdrInterface) + "'",
        [&systemBus](sdbusplus::message_t& msg) {
            sdrHandler(sdrsensor, msg, systemBus);
        });

    // Watch for entity-manager to remove configuration interfaces
    // so the corresponding sensors can be removed.
    auto ifaceRemovedMatch = std::make_shared<sdbusplus::bus::match_t>(
        static_cast<sdbusplus::bus_t&>(*systemBus),
        "type='signal',member='InterfacesRemoved',arg0path='" +
            std::string(inventoryPath) + "/'",
        [](sdbusplus::message_t& msg) { interfaceRemoved(msg, sensors); });

    setupManufacturingModeMatch(*systemBus);
    io.run();
    return 0;
}
