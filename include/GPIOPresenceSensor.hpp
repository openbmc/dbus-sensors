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

static constexpr const int  pollRateDefault = 10;

static constexpr const char* service = "xyz.openbmc_project.GPIOPresence";

namespace Properties
{
static constexpr const char* propertyName = "Name";
static constexpr const char* propertyGpioLine = "GpioLine";
static constexpr const char* propertyPolarity = "Polarity";
static constexpr const char* propertyPresent = "Present";
static constexpr const char* propertyPollRate = "PollRate";
static constexpr const char* propertyFaultLedGroup = "FaultLedGroup";
static constexpr const char* propertyClass = "Class";
} // namespace Properties

namespace interfaces
{
static constexpr const char* emGPIOCableSensingIfc =
    "xyz.openbmc_project.Configuration.GPIOPresence";
static constexpr const char* cableIfc = "xyz.openbmc_project.Inventory.Item.Cable";
static constexpr const char* statusIfc = "xyz.openbmc_project.Inventory.Item";
} // namespace interfaces

static constexpr const char* inventoryObjPath =
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
    // Update loop polling rate.
    int pollRate;
    // Assert on led groups if the presence signal is missing.
    std::vector<std::string> faultLedGroup;
};

using DBusVariant = std::variant<int, std::string, std::vector<std::string>, bool>;
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
    GPIOPresence(std::shared_ptr<sdbusplus::asio::connection> bus) 
        : bus(bus), pollRate(pollRateDefault) {}

    // Add a dbus object to the reference list.
    // @params inventoryIfc: pointer to object inventory interface.
    // @params statusIfc: pointer to object status interface.
    // @params objPath: the dbus object path.
    // @params config: EM config
    void addObj(std::unique_ptr<sdbusplus::asio::dbus_interface> inventoryIfc,
                std::unique_ptr<sdbusplus::asio::dbus_interface> statusIfc,
                std::string_view objPath, const Config& config);

    // Remove a object from the object reference list.
    void removeObj(std::string_view objPath);

    // Check if a object is included in the obj->iface map already.
    bool hasObj(std::string_view objPath);

    // periodically print heartbeat statement for debug purpose.
    void heartBeat(int interval);

    // Start the monitor.
    void startUpdateLoop(bool forceUpdate);

    // Set the minimum polling rate.
    void setMinPollRate(int newRate) {
      pollRate = std::min(pollRate, newRate);
    }

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
    // default polling rate.
    int pollRate;
};

} // namespace gpiopresencesensing
