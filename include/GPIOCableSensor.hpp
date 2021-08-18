#include <boost/asio.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/bus/match.hpp>

#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace gpiocablesensing
{

static constexpr char service[] = "xyz.openbmc_project.GPIOCableSensing";

namespace interfaces
{
static constexpr char emGPIOCableSensingIfc[] =
    "xyz.openbmc_project.Configuration.GPIOCableSensing";
static constexpr char cableIfc[] = "xyz.openbmc_project.Inventory.Item.Cable";
static constexpr char connectorIfc[] =
    "xyz.openbmc_project.Inventory.Item.Connector";
static constexpr char operationalStatusIfc[] =
    "xyz.openbmc_project.State.Decorator.OperationalStatus";
} // namespace interfaces

static constexpr char inventoryObjPath[] =
    "/xyz/openbmc_project/inventory/item/";
namespace Properties
{
static constexpr char propertyName[] = "Name";
static constexpr char propertyGpioLine[] = "GpioLine";
static constexpr char propertyPolarity[] = "Polarity";
static constexpr char propertyFunctional[] = "Functional";
static constexpr char propertyFaultLedGroup[] = "FaultLedGroup";
} // namespace Properties

struct Config
{
    std::string name;
    // interface gpio pin
    std::string gpioLine;
    // GPIO Polarity
    bool activeLow;
    // interface OperationalStatus
    bool functional;
    // Assert on led groups if the cable is not functional
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
        auto gpioLine = getDbusProperty<std::string>(
            properties, Properties::propertyGpioLine);
        auto polarity = getDbusProperty<std::string>(
            properties, Properties::propertyPolarity);
        auto faultLedGroup = getDbusProperty<std::vector<std::string>>(
            properties, Properties::propertyFaultLedGroup);
        return {name, gpioLine, polarity == "active_low", /*functional*/ false,
                faultLedGroup};
    }

    std::string interface;
    std::unique_ptr<sdbusplus::bus::match::match> ifcAdded;
    std::unique_ptr<sdbusplus::bus::match::match> ifcRemoved;
};

} // namespace gpiocablesensing
