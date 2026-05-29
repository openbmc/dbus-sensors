#pragma once

#include <SensorPaths.hpp>
#include <Utils.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using namespace phosphor::logging;
using namespace sdbusplus;

constexpr const char* sensorStateInterface = "xyz.openbmc_project.Sensor.State";
constexpr auto PROP_INTF = "org.freedesktop.DBus.Properties";
constexpr auto METHOD_GET = "Get";
constexpr auto METHOD_GET_ALL = "GetAll";
constexpr auto METHOD_SET = "Set";

constexpr const char* warningInterface =
    "xyz.openbmc_project.Sensor.Threshold.Warning";
constexpr const char* criticalInterface =
    "xyz.openbmc_project.Sensor.Threshold.Critical";

using value = std::variant<uint8_t, uint16_t, std::string>;
using property = std::string;
using sensorValue = std::variant<int64_t, double, std::string, bool>;
using propertyMap = std::map<property, sensorValue>;
enum class IPMIThresholds
{
    warningLow,
    warningHigh,
    criticalLow,
    criticalHigh,
};

enum class HostState
{
    Running,
    Off,
    Unknown,
};

struct Discrete
{
    Discrete(const std::string& name, const std::string& configurationPath,
             std::shared_ptr<sdbusplus::asio::connection>& conn) :
        name(sensor_paths::escapePathForDbus(name)),
        configurationPath(configurationPath), dbusConnection(conn)

    {}
    virtual ~Discrete() = default;
    std::string name;
    std::string configurationPath;

    std::shared_ptr<sdbusplus::asio::dbus_interface> sensorInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> association;

    uint16_t state = 0;

    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
#ifdef FEATURE_APISENSOR_SUPPORT
    bool internalSet = false;
    // This member variable provides a hook that can be used to receive
    // notification whenever this Sensor's value is externally set via D-Bus.
    // If interested, assign your own lambda to this variable, during
    // construction of your Sensor subclass. See ExternalSensor for example.
    std::function<void(uint16_t newState)> externalSetHook;
#endif
    int updateState(std::shared_ptr<sdbusplus::asio::dbus_interface>& interface,
                    const uint16_t& newState)
    {
#ifdef FEATURE_APISENSOR_SUPPORT
        internalSet = true;
#endif
        if (interface && !(interface->set_property("State", newState)))
        {
            std::cerr << "error setting State \n";
        }
#ifdef FEATURE_APISENSOR_SUPPORT
        internalSet = false;
#endif
        return 1;
    }

    int setSensorState(const uint16_t& newState, uint16_t& oldState)
    {
#ifdef FEATURE_APISENSOR_SUPPORT
        if (!internalSet)
        {
            // Trigger the hook, as an external set has just happened
            if (externalSetHook)
            {
                // Will throw if fails (state will not be updated)
                externalSetHook(newState);
            }
        }
#endif
        state = newState;
        oldState = newState;
        return 1;
    }

    void setInitialProperties()
    {
        createAssociation(association, configurationPath);

        sensorInterface->register_property(
            "State", state,
            [this](const uint16_t& newState, uint16_t& oldState) {
                return setSensorState(newState, oldState);
            });

        if (!sensorInterface->initialize())
        {
            std::cerr << "error initializing state interface\n";
        }
    }

    std::shared_ptr<sdbusplus::bus::match_t> setupDbusMatch(
        std::string path, std::string interface,
        std::function<void(sdbusplus::message_t& msg)>&& statusCallback)
    {
        return std::make_shared<sdbusplus::bus::match_t>(
            static_cast<sdbusplus::bus_t&>(*dbusConnection),
            "type='signal',interface='" +
                std::string("org.freedesktop.DBus.Properties") + "',path='" +
                std::string(path) + "',arg0='" + std::string(interface) + "'",
            std::move(statusCallback));
    }

    uint8_t getHostStatus(std::shared_ptr<sdbusplus::asio::connection> conn)
    {
        std::string pwrStatus;
        value variant;
        try
        {
            auto method =
                conn->new_method_call(power::busname, power::path,
                                      properties::interface, properties::get);
            method.append(power::interface, "CurrentHostState");
            auto reply = conn->call(method);
            reply.read(variant);
            pwrStatus = std::get<std::string>(variant);
        }
        catch (sdbusplus::exception_t& e)
        {
            std::cerr << "Failed to get getHostStatus Value";
            return (static_cast<uint8_t>(HostState::Unknown));
        }
        if (pwrStatus == "xyz.openbmc_project.State.Host.HostState.Running")
        {
            return (static_cast<uint8_t>(HostState::Running));
        }
        else if (pwrStatus == "xyz.openbmc_project.State.Host.HostState.Off")
        {
            return (static_cast<uint8_t>(HostState::Off));
        }
        return (static_cast<uint8_t>(HostState::Unknown));
    }
};

inline std::string getService(const std::string& intf, const std::string& path)
{
    auto bus = bus::new_default_system();
    auto mapper =
        bus.new_method_call("xyz.openbmc_project.ObjectMapper",
                            "/xyz/openbmc_project/object_mapper",
                            "xyz.openbmc_project.ObjectMapper", "GetObject");

    mapper.append(path);
    mapper.append(std::vector<std::string>({intf}));

    std::map<std::string, std::vector<std::string>> response;

    try
    {
        auto responseMsg = bus.call(mapper);

        responseMsg.read(response);
    }
    catch (const sdbusplus::exception::exception& ex)
    {
        log<level::ERR>("ObjectMapper call failure",
                        entry("WHAT=%s", ex.what()));
        throw;
    }

    if (response.begin() == response.end())
    {
        throw std::runtime_error("Unable to find Object: " + path);
    }

    return response.begin()->first;
}

inline int8_t monitorThreshold(const std::string sensorName,
                               const std::string sensorObjectPath)
{
    std::string objectPath = sensorObjectPath;
    int8_t status = 0;
    boost::asio::io_context io;
    auto conn = std::make_shared<sdbusplus::asio::connection>(io);
    objectPath = objectPath + sensorName;
    std::string service;
    // warning interface
    try
    {
        service = getService(warningInterface, objectPath.c_str());
        propertyMap warningMap;
        auto method = conn->new_method_call(service.c_str(), objectPath.c_str(),
                                            PROP_INTF, METHOD_GET_ALL);
        method.append(warningInterface);
        auto reply = conn->call(method);
        if (reply.is_method_error())
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Failed to get all properties");
        }
        reply.read(warningMap);
        auto findWarningHigh = warningMap.find("WarningAlarmLow");
        if (findWarningHigh != warningMap.end())
        {
            if (std::get<bool>(warningMap.at("WarningAlarmLow")))
            {
                status = status |
                         (1 << static_cast<int8_t>(IPMIThresholds::warningLow));
            }
        }
    }
    catch (sdbusplus::exception_t& e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Failed to fetch",
            phosphor::logging::entry("EXCEPTION=%s", e.what()));
    }

    // critical interface
    try
    {
        service = getService(criticalInterface, objectPath.c_str());
        propertyMap criticalMap;
        auto method = conn->new_method_call(service.c_str(), objectPath.c_str(),
                                            PROP_INTF, METHOD_GET_ALL);
        method.append(criticalInterface);
        auto reply = conn->call(method);
        if (reply.is_method_error())
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Failed to get all properties");
        }
        reply.read(criticalMap);
        auto findCriticalHigh = criticalMap.find("CriticalAlarmLow");

        if (findCriticalHigh != criticalMap.end())
        {
            if (std::get<bool>(criticalMap.at("CriticalAlarmLow")))
            {
                status =
                    status |
                    (1 << static_cast<int8_t>(IPMIThresholds::criticalLow));
            }
        }
    }
    catch (sdbusplus::exception_t& e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Failed to fetch",
            phosphor::logging::entry("EXCEPTION=%s", e.what()));
    }
    return status;
}
