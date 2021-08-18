#include <GPIOPresenceSensor.hpp>

#include <iostream>

namespace gpiopresencesensing
{
static std::unique_ptr<sdbusplus::bus::match::match> ifcAdded;
static std::unique_ptr<sdbusplus::bus::match::match> ifcRemoved;

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
    auto gpioLine =
        getDbusProperty<std::string>(properties, Properties::propertyGpioLine);
    auto polarity =
        getDbusProperty<std::string>(properties, Properties::propertyPolarity);
    auto faultLedGroup = getDbusProperty<std::vector<std::string>>(
        properties, Properties::propertyFaultLedGroup);
    int pollRate = pollRateDefault;
    try {
      pollRate = getDbusProperty<int>(
        properties, Properties::propertyPollRate);
    } catch (std::exception&) {
      // It is ok for poll rate not defined.
    }
    return {name,
            className,
            gpioLine,
            polarity == "active_low",
            /*present*/ false,
            pollRate,
            faultLedGroup};
}

void onInterfaceAdded(sdbusplus::asio::connection* conn,
                      OnInterfaceAddedCallback&& cb)
{
    std::function<void(sdbusplus::message::message & msg)> handler =
        [callback = cb](sdbusplus::message::message& msg) {
            sdbusplus::message::object_path objPath;
            IfcToProperties ifcAndProperties;
            msg.read(objPath, ifcAndProperties);
            auto found =
                ifcAndProperties.find(interfaces::emGPIOCableSensingIfc);
            if (found != ifcAndProperties.end())
            {
                Config config;
                try
                {
                    config = getConfig(found->second);
                    callback(objPath.str, found->first, config);
                }
                catch (std::exception& e)
                {
                    std::cerr << "Incomplete config found: " << e.what()
                              << " obj = " << objPath.str << std::endl;
                }
            }
        };

    // call the user callback for all the device that is already available
    auto msg = conn->new_method_call("xyz.openbmc_project.EntityManager", "/",
                                     "org.freedesktop.DBus.ObjectManager",
                                     "GetManagedObjects");
    ManagedObjectType objects;
    try
    {
        auto reply = conn->call(msg);
        ManagedObjectType managedObjs;
        reply.read(managedObjs);
        for (auto& obj : managedObjs)
        {
            auto& item = obj.second;
            auto found = item.find(interfaces::emGPIOCableSensingIfc);
            if (found != item.end())
            {
                Config config;
                try
                {
                    config = getConfig(found->second);
                    cb(obj.first.str, found->first, config);
                }
                catch (std::exception& e)
                {
                    std::cerr << "Incomplete config found: " << e.what()
                              << " obj = " << obj.first.str << std::endl;
                }
            }
        }
    }
    catch (std::exception& ex)
    {
        std::cerr << "got exception " << ex.what() << std::endl;
        // that is ok. Looks like nothing is populated onto the dbus yet.
        // So convert this operation into a no op.
    }

    sdbusplus::bus::match::match match(
        static_cast<sdbusplus::bus::bus&>(*conn),
        sdbusplus::bus::match::rules::interfacesAdded() +
            sdbusplus::bus::match::rules::sender(
                "xyz.openbmc_project.EntityManager"),
        handler);
    ifcAdded = std::make_unique<sdbusplus::bus::match::match>(std::move(match));
}

void onInterfaceRemoved(sdbusplus::asio::connection* conn,
                        OnInterfaceRemovedCallback&& cb)
{
    // Listen to the interface removed event.
    std::function<void(sdbusplus::message::message & msg)> handler =
        [callback = std::move(cb)](sdbusplus::message::message& msg) {
            sdbusplus::message::object_path objPath;
            IfcToProperties ifcAndProperties;
            msg.read(objPath, ifcAndProperties);
            callback(objPath.str);
        };
    sdbusplus::bus::match::match match(
        static_cast<sdbusplus::bus::bus&>(*conn),
        sdbusplus::bus::match::rules::interfacesRemoved() +
            sdbusplus::bus::match::rules::sender(
                "xyz.openbmc_project.EntityManager"),
        handler);
    ifcRemoved =
        std::make_unique<sdbusplus::bus::match::match>(std::move(match));
}
} // namespace gpiopresencesensing

void startMain(
    int delay, std::shared_ptr<sdbusplus::asio::connection> bus,
    std::shared_ptr<gpiopresencesensing::GPIOPresence> controller) {
  static bool isRunning = false;
  static boost::asio::steady_timer timer(bus->get_io_context());
  if (isRunning) {
    return;
  }
  timer.cancel();
  timer.expires_after(std::chrono::seconds(delay));
  timer.async_wait([delay, bus, controller](const boost::system::error_code& ec) {
    if (ec == boost::asio::error::operation_aborted)
    {
      std::cout << "Delaying update loop" << std::endl;
      return;
    }
    std::cout << "Update loop started" << std::endl;
    controller->startUpdateLoop(/*forceUpdate=*/true);
    isRunning = true;
  });
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name(gpiopresencesensing::service);
    sdbusplus::asio::object_server objectServer(systemBus);

    std::shared_ptr<gpiopresencesensing::GPIOPresence> controller =
        std::make_shared<gpiopresencesensing::GPIOPresence>(systemBus);

    // Wait for config from EM.
    // Once a new config is posted. We will create the obj on dbus.
    gpiopresencesensing::onInterfaceAdded(
        systemBus.get(), [&controller, &systemBus, &objectServer](
                             std::string_view, std::string_view,
                             const gpiopresencesensing::Config& config) {
            std::string objPath =
                std::string(gpiopresencesensing::inventoryObjPath) +
                config.name;
            std::cout << "New config received " << objPath << std::endl;
            if (!controller->hasObj(objPath)) {              
              const char* inventoryClassIfcName =
                  gpiopresencesensing::classToInterface(config.className);
              if (inventoryClassIfcName == nullptr)
              {
                  std::cerr << "Undefined class name for " << config.name;
                  return;
              }
              // inventory class Ifc
              auto inventoryClassIfc = objectServer.add_unique_interface(
                  objPath, std::string(inventoryClassIfcName));
              inventoryClassIfc->initialize();
              // Status
              auto statusIfc = objectServer.add_unique_interface(
                  objPath, gpiopresencesensing::interfaces::statusIfc);
              statusIfc->register_property("PrettyName", config.name);
              statusIfc->register_property(
                  gpiopresencesensing::Properties::propertyPresent, false);
              statusIfc->initialize();
              controller->addObj(std::move(inventoryClassIfc),
                                 std::move(statusIfc), objPath, config);
              controller->setMinPollRate(config.pollRate);
            }
            // It is possible there are more EM config in the pipeline.
            // Therefore, delay the main loop by 10 seconds to wait for more
            // configs.
            startMain(/*delay=*/10, systemBus, controller);
        });

    gpiopresencesensing::onInterfaceRemoved(
        systemBus.get(), [&controller](std::string_view objPath) {
            controller->removeObj(std::string(objPath));
        });

    controller->heartBeat(/*interval=*/30);
    io.run();
}
