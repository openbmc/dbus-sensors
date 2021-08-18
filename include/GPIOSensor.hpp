#include <boost/asio.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/bus/match.hpp>

#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace GPIOSensing
{

static constexpr char service[] = "xyz.openbmc_project.GPIOSensing";

namespace Interfaces
{
static constexpr char EMGPIOSensingIfc[] =
    "xyz.openbmc_project.Configuration.GPIOSensing";
static constexpr char CableIfc[] = "xyz.openbmc_project.Inventory.Item.Cable";
static constexpr char ConnectorIfc[] = "xyz.openbmc_project.Inventory.Item.Connector";
static constexpr char OperationalStatusIfc[] =
    "xyz.openbmc_project.State.Decorator.OperationalStatus";
} // namespace Interfaces

static constexpr char InventoryObjPath[] =
    "/xyz/openbmc_project/inventory/item/";
namespace Properties
{
static constexpr char PropertyName[] = "Name";
static constexpr char PropertyGpioLine[] = "GpioLine";
static constexpr char PropertyIsActiveLow[] = "IsActiveLow";
static constexpr char PropertyFunctional[] = "Functional";
} // namespace Properties

struct Config
{
    std::string name;
    // interface gpio pin
    std::string gpioLine;
   // GPIO isActiveLow or isActiveHigh
    bool isActiveLow;
    // interface OperationalStatus
    bool functional;
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
    GPIOSensorConfigListener(std::string intf) : interface(intf)
    {}

    void onInterfaceAdded(sdbusplus::asio::connection* conn,
                          OnInterfaceAddedCallback&& cb);

    void onInterfaceRemoved(sdbusplus::asio::connection* conn,
                            OnInterfaceRemovedCallback&& cb);

  private:
    template <typename T>
    T fromDbusProperty(const DBusProperties& properties, const std::string& key)
    {
        T ret{};
        auto found = properties.find(key);
        if (found != properties.end())
        {
            ret = std::get<T>(found->second);
        }
        return ret;
    }

    Config toConfig(DBusProperties& properties)
    {
        return {
            /*name*/ fromDbusProperty<std::string>(properties,
                                                   Properties::PropertyName),
            /*GpioLine*/
            fromDbusProperty<std::string>(properties,
                                          Properties::PropertyGpioLine),
            /*isActiveLow*/
            fromDbusProperty<bool>(properties, Properties::PropertyIsActiveLow),
            /*functional*/ false};
    }

    std::string interface;
    std::unique_ptr<sdbusplus::bus::match::match> ifcAdded;
    std::unique_ptr<sdbusplus::bus::match::match> ifcRemoved;
};

} // namespace GPIOSensing
