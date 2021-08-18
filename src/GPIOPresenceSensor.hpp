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
#include <unordered_map>
#include <variant>
#include <vector>

namespace gpio_presence_sensing
{

static const unsigned pollRateDefault = 10;

static constexpr const char* service = "xyz.openbmc_project.GpioSensor";
static constexpr const char* inventoryObjPath =
    "/xyz/openbmc_project/inventory/item/";

namespace properties
{
static constexpr const char* propertyName = "Name";
static constexpr const char* propertyGpioLine = "GpioLine";
static constexpr const char* propertyPolarity = "Polarity";
static constexpr const char* propertyPresent = "Present";
static constexpr const char* propertyPollRate = "PollRate";
} // namespace properties

namespace interfaces
{
static constexpr const char* emGPIOCableSensingIfc =
    "xyz.openbmc_project.Configuration.GPIOBasedItemPresence";
static constexpr const char* statusIfc = "xyz.openbmc_project.Inventory.Item";
} // namespace interfaces

struct Config
{
    std::string name;
    // interface gpio pin.
    std::string gpioLine;
    // GPIO Polarity.
    bool activeLow{};
    // Presence signal.
    bool present{};
    // Update loop polling rate.
    unsigned pollRate{};
};

// Actively listen to the config information from EntityManager and calls the
// callback function once a config is available.
class GPIOPresence : public std::enable_shared_from_this<GPIOPresence>
{
  public:
    explicit GPIOPresence(
        const std::shared_ptr<sdbusplus::asio::connection>& bus) :
        bus(bus),
        timer(bus->get_io_context()), pollRate(pollRateDefault)
    {}

    // Add a dbus object to the reference list.
    // @params statusIfc: pointer to object status interface.
    // @params objPath: the dbus object path.
    // @params config: EM config
    void addObj(std::unique_ptr<sdbusplus::asio::dbus_interface>& statusIfc,
                const std::string& objPath, const Config& config);

    // Remove a object from the object reference list.
    void removeObj(const std::string& objPath);

    // Check if a object is included in the obj->iface map already.
    bool hasObj(const std::string& objPath);

    // Start the monitor.
    void startUpdateLoop(bool forceUpdate);

    // Set the minimum polling rate.
    void setMinPollRate(unsigned newRate);

  private:
    struct ObjIfaces
    {
        std::unique_ptr<sdbusplus::asio::dbus_interface> statusIfc;
        Config config;
    };

    void addInputLine(const std::string& lineLabel);

    int readLine(const std::string& lineLabel);

    // dbus connection.
    std::shared_ptr<sdbusplus::asio::connection> bus;
    boost::asio::steady_timer timer;
    // Reference to dbus object interfaces.
    std::unordered_map<std::string, ObjIfaces> objIfaces;
    // Reference to gpioLines.
    std::unordered_map<std::string, ::gpiod::line> gpioLines;
    // default polling rate.
    unsigned pollRate;
};

} // namespace gpio_presence_sensing
