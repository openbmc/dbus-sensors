#pragma once

#include <boost/asio.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/bus/match.hpp>

#include <functional>
#include <iostream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <variant>
#include <vector>

namespace gpiopresencesensing
{

static constexpr char service[] = "xyz.openbmc_project.GPIOPresence";

namespace Properties
{
static constexpr char propertyName[] = "Name";
static constexpr char propertyGpioLine[] = "GpioLine";
static constexpr char propertyPolarity[] = "Polarity";
static constexpr char propertyPresent[] = "Present";
static constexpr char propertyFaultLedGroup[] = "FaultLedGroup";
static constexpr char propertyClass[] = "Class";
} // namespace Properties

namespace interfaces
{
static constexpr char emGPIOCableSensingIfc[] =
    "xyz.openbmc_project.Configuration.GPIOPresence";
static constexpr char cableIfc[] = "xyz.openbmc_project.Inventory.Item.Cable";
static constexpr char statusIfc[] =
    "xyz.openbmc_project.Inventory.Item";
} // namespace interfaces

static constexpr char inventoryObjPath[] =
    "/xyz/openbmc_project/inventory/item/";

struct Config
{
    std::string name;
    std::string className;
    // interface gpio pin.
    std::string gpioLine;
    // GPIO Polarity.
    bool activeLow;
    // Presence signal.
    bool present;
    // Assert on led groups if the presence signal is not missing.
    std::vector<std::string> faultLedGroup;
};

using DBusVariant = std::variant<std::string, std::vector<std::string>, bool>;
using DBusProperties = std::unordered_map<std::string, DBusVariant>;
using IfcToProperties = std::unordered_map<std::string, DBusProperties>;
using ManagedObjectType =
    std::unordered_map<sdbusplus::message::object_path, IfcToProperties>;
using OnInterfaceAddedCallback =
    std::function<void(std::string_view, std::string_view, const Config&)>;
using OnInterfaceRemovedCallback = std::function<void(std::string_view)>;

// Helper function to convert a class type from configure to interface
const char* classToInterface(std::string_view className);

// Actively listen to the config information from EntityManager and calls the
// callback function once a config is available.
class GPIOSensorConfigListener
{
  public:
    GPIOSensorConfigListener(const std::string& intf) : interface(intf)
    {}

    // Monitor xyz.openbmc_project.Configuration.GPIOCableSensing
    // @param[in] cb: the callback to invoke when interface added
    void onInterfaceAdded(sdbusplus::asio::connection* conn,
                          OnInterfaceAddedCallback&& cb);

    // Monitor xyz.openbmc_project.Configuration.GPIOCableSensing
    // @param[in] cb: the callback to invoke when interface removed
    void onInterfaceRemoved(sdbusplus::asio::connection* conn,
                            OnInterfaceRemovedCallback&& cb);

  private:
    // Get a dbus property from properties
    // @param[in] properties: dbus properties
    // @param[in] key: property key
    template <typename T>
    T getDbusProperty(const DBusProperties& properties, const std::string& key)
    {
        T ret;
        auto found = properties.find(key);
        if (found != properties.end())
        {
            ret = std::get<T>(found->second);
        }
        else
        {
            std::string err = std::string("key ") + key + " not found";
            throw std::invalid_argument(err);
        }
        return ret;
    }

    // Helper function to convert dbus property to struct
    // @param[in] properties: dbus properties
    Config getConfig(const DBusProperties& properties)
    {
        auto name =
            getDbusProperty<std::string>(properties, Properties::propertyName);
        auto className =
            getDbusProperty<std::string>(properties, Properties::propertyClass);
        auto gpioLine = getDbusProperty<std::string>(
            properties, Properties::propertyGpioLine);
        auto polarity = getDbusProperty<std::string>(
            properties, Properties::propertyPolarity);
        auto faultLedGroup = getDbusProperty<std::vector<std::string>>(
            properties, Properties::propertyFaultLedGroup);
        return {name,
                className,
                gpioLine,
                polarity == "active_low",
                /*present*/ false,
                faultLedGroup};
    }

    std::string interface;
    std::unique_ptr<sdbusplus::bus::match::match> ifcAdded;
    std::unique_ptr<sdbusplus::bus::match::match> ifcRemoved;
};

} // namespace gpiopresencesensing
