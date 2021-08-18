#pragma once

#include <boost/asio.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
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
static constexpr char statusIfc[] = "xyz.openbmc_project.Inventory.Item";
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
class GPIOPresence
{
  public:
    GPIOPresence(std::shared_ptr<sdbusplus::asio::connection> bus) : bus(bus)
    {}

    // Add a dbus object to the reference list.
    // @params inventoryIfc: pointer to object inventory interface.
    // @params statusIfc: pointer to object status interface.
    // @params objPath: the dbus object path.
    // @params config: EM config
    void addObj(std::unique_ptr<sdbusplus::asio::dbus_interface> inventoryIfc,
                std::unique_ptr<sdbusplus::asio::dbus_interface> statusIfc,
                const std::string& objPath, const Config& config);

    // Remove a object from the object reference list.
    void removeObj(const std::string& objPath);

    // Start the monitor.
    void startUpdateLoop(bool forceUpdate);

  private:
    struct ObjIfaces
    {
        std::unique_ptr<sdbusplus::asio::dbus_interface> inventoryClassIfc;
        std::unique_ptr<sdbusplus::asio::dbus_interface> statusIfc;
        Config config;
    };

    // @param faultLedAssert: indicator for if a fault led need to be asserted.
    void driveFaultLedGroup(
        const boost::container::flat_map<std::string, bool>& faultLedAssert);

    // dbus connection.
    std::shared_ptr<sdbusplus::asio::connection> bus;
    // Reference to dbus object interfaces.
    std::unordered_map<std::string, ObjIfaces> objIfaces;
};

} // namespace gpiopresencesensing
