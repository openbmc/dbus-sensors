#pragma once

#include <boost/asio.hpp>
#include <boost/container/flat_map.hpp>
#include <gpiod.hpp>
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

static const int pollRateDefault = 10;

static constexpr inline const char* service = "xyz.openbmc_project.GPIOStatus";
static constexpr inline const char* inventoryObjPath =
    "/xyz/openbmc_project/inventory/item/";

namespace Properties
{
static constexpr inline const char* propertyName = "Name";
static constexpr inline const char* propertyGpioLine = "GpioLine";
static constexpr inline const char* propertyPolarity = "Polarity";
static constexpr inline const char* propertyPresent = "Present";
static constexpr inline const char* propertyPollRate = "PollRate";
static constexpr inline const char* propertyAssociationPath = "AssociationPath";
static constexpr inline const char* propertyAssociationForward = "AssociationForward";
static constexpr inline const char* propertyAssociationReverse = "AssociationReverse";
} // namespace Properties

namespace interfaces
{
static constexpr inline const char* emGPIOCableSensingIfc =
    "xyz.openbmc_project.Configuration.GPIOStatus";
static constexpr inline const char* statusIfc =
    "xyz.openbmc_project.GPIOStatus";
} // namespace interfaces

struct Config
{
    std::string name;
    // interface gpio pin.
    std::string gpioLine;
    // GPIO Polarity.
    bool activeLow;
    // Presence signal.
    bool present;
    // (Optional) Update loop polling rate.
    int pollRate;
    // (Optional) Association
    bool generateAssociation;
    std::string associationPath;
    std::string associationForward;
    std::string associationReverse;
    // (Internal) Parent path
    std::string parentPath;
};

// Actively listen to the config information from EntityManager and calls the
// callback function once a config is available.
class GPIOPresence
{
  public:
    GPIOPresence(std::shared_ptr<sdbusplus::asio::connection> bus) :
        bus(bus), timer(bus->get_io_context()), pollRate(pollRateDefault)
    {}

    // Add a dbus object to the reference list.
    // @params statusIfc: pointer to object status interface.
    // @params associationIfc: Optional pointer to object association interface
    // @params objPath: the dbus object path.
    // @params config: EM config
    void addObj(std::unique_ptr<sdbusplus::asio::dbus_interface> statusIfc,
                std::unique_ptr<sdbusplus::asio::dbus_interface> associationIfc,
                std::string_view objPath, const Config& config);

    // Remove a object from the object reference list.
    void removeObj(std::string_view objPath);

    // Check if a object is included in the obj->iface map already.
    bool hasObj(std::string_view objPath);

    // Start the monitor.
    void startUpdateLoop(bool forceUpdate);

    // Set the minimum polling rate.
    void setMinPollRate(int newRate)
    {
        pollRate = std::min(pollRate, newRate);
    }

  private:
    struct ObjIfaces
    {
        std::unique_ptr<sdbusplus::asio::dbus_interface> statusIfc;
        std::unique_ptr<sdbusplus::asio::dbus_interface> associationIfc;
        Config config;
    };

    void addInputLine(std::string_view lineLabel);

    int readLine(std::string_view lineLabel);

    // dbus connection.
    std::shared_ptr<sdbusplus::asio::connection> bus;
    boost::asio::steady_timer timer;
    // Reference to dbus object interfaces.
    std::unordered_map<std::string, ObjIfaces> objIfaces;
    // Reference to gpioLines.
    std::unordered_map<std::string, ::gpiod::line> gpioLines;
    // default polling rate.
    int pollRate;
};

} // namespace gpiopresencesensing
