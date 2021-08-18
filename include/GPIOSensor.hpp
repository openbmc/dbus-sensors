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
static constexpr char OperationalStatusIfc[] =
    "xyz.openbmc_project.State.Decorator.OperationalStatus";
static constexpr char AssetIfc[] =
    "xyz.openbmc_project.Inventory.Decorator.Asset";
static constexpr char AssetTagIfc[] =
    "xyz.openbmc_project.Inventory.Decorator.AssetTag";
} // namespace Interfaces

static constexpr char InventoryObjPath[] =
    "/xyz/openbmc_project/inventory/item/cable/";
namespace Properties
{
static constexpr char PropertyName[] = "Name";
static constexpr char PropertyPinLabel[] = "PinLabel";
static constexpr char PropertyPartNumber[] = "PartNumber";
static constexpr char PropertySerialNumber[] = "SerialNumber";
static constexpr char PropertyManufacturer[] = "Manufacturer";
static constexpr char PropertyBuildDate[] = "BuildDate";
static constexpr char PropertyModel[] = "Model";
static constexpr char PropertySubModel[] = "SubModel";
static constexpr char PropertySparePartNumber[] = "SparePartNumber";
static constexpr char PropertyAssetTag[] = "AssetTag";
static constexpr char PropertyIsActiveLow[] = "IsActiveLow";
static constexpr char PropertyFunctional[] = "Functional";
} // namespace Properties

struct Config
{
    std::string name;
    // interface gpio pin
    std::string pinLabel;
    // interface Asset
    std::string partNumber;
    std::string serialNumber;
    std::string manufacturer;
    std::string buildDate;
    std::string model;
    std::string subModel;
    std::string sparePartNumber;
    // interface AssetTag
    std::string assetTag;
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
            /*pinLabel*/
            fromDbusProperty<std::string>(properties,
                                          Properties::PropertyPinLabel),
            /*partNumber*/
            fromDbusProperty<std::string>(properties,
                                          Properties::PropertyPartNumber),
            /*serialNumber*/
            fromDbusProperty<std::string>(properties,
                                          Properties::PropertySerialNumber),
            /*manufacturer*/
            fromDbusProperty<std::string>(properties,
                                          Properties::PropertyManufacturer),
            /*buildDate*/
            fromDbusProperty<std::string>(properties,
                                          Properties::PropertyBuildDate),
            /*model*/
            fromDbusProperty<std::string>(properties,
                                          Properties::PropertyModel),
            /*subModel*/
            fromDbusProperty<std::string>(properties,
                                          Properties::PropertySubModel),
            /*sparePartNumber*/
            fromDbusProperty<std::string>(properties,
                                          Properties::PropertySparePartNumber),
            /*assetTag*/
            fromDbusProperty<std::string>(properties,
                                          Properties::PropertyAssetTag),
            /*isActiveLow*/
            fromDbusProperty<bool>(properties, Properties::PropertyIsActiveLow),
            /*functional*/ false};
    }

    std::string interface;
    std::unique_ptr<sdbusplus::bus::match::match> ifcAdded;
    std::unique_ptr<sdbusplus::bus::match::match> ifcRemoved;
};

} // namespace GPIOSensing
