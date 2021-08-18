#include <GPIOCableSensor.hpp>

namespace gpiocablesensing
{

void GPIOSensorConfigListener::onInterfaceAdded(
    sdbusplus::asio::connection* conn, OnInterfaceAddedCallback&& cb)
{
    std::function<void(sdbusplus::message::message & msg)> handler =
        [this, callback = cb](sdbusplus::message::message& msg) {
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

void GPIOSensorConfigListener::onInterfaceRemoved(
    sdbusplus::asio::connection* conn, OnInterfaceRemovedCallback&& cb)
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

} // namespace gpiocablesensing
