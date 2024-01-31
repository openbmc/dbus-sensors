#include "MCTPReactor.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/message/native_types.hpp>

PHOSPHOR_LOG2_USING;

static constexpr const char* const mctpInterfaceInterface =
    "xyz.openbmc_project.Configuration.MCTPInterface";

static constexpr const char* const mctpDeviceInterface =
    "xyz.openbmc_project.Configuration.MCTPDevice";

class DBusAssociationServer : public AssociationServer
{
  public:
    DBusAssociationServer() = delete;
    DBusAssociationServer(const DBusAssociationServer&) = delete;
    DBusAssociationServer(DBusAssociationServer&&) = delete;
    explicit DBusAssociationServer(
        const std::shared_ptr<sdbusplus::asio::connection>& connection) :
        server(connection)
    {
        server.add_manager("/xyz/openbmc_project/mctp");
    }
    ~DBusAssociationServer() override = default;
    DBusAssociationServer& operator=(const DBusAssociationServer&) = delete;
    DBusAssociationServer& operator=(DBusAssociationServer&&) = delete;

    void associate(const std::string& path,
                   const std::vector<Association>& associations) override
    {
        auto [entry, _] = objects.emplace(
            path, server.add_interface(path, association::interface));
        std::shared_ptr<sdbusplus::asio::dbus_interface> iface = entry->second;
        iface->register_property("Associations", associations);
        iface->initialize();
    }

    void disassociate(const std::string& path) override
    {
        const auto entry = objects.find(path);
        if (entry == objects.end())
        {
            throw std::logic_error(std::format(
                "Attempted to untrack path that was not tracked: {}", path));
        }
        std::shared_ptr<sdbusplus::asio::dbus_interface> iface = entry->second;
        server.remove_interface(entry->second);
        objects.erase(entry);
    }

  private:
    std::shared_ptr<sdbusplus::asio::connection> connection;
    sdbusplus::asio::object_server server;
    std::map<std::string, std::shared_ptr<sdbusplus::asio::dbus_interface>>
        objects;
};

static void addInventory(const std::shared_ptr<MCTPReactor>& reactor,
                         sdbusplus::message_t& msg)
{
    auto [path,
          exposed] = msg.unpack<sdbusplus::message::object_path, SensorData>();
    auto interfaceEntry = exposed.find(mctpInterfaceInterface);
    if (interfaceEntry != exposed.end())
    {
        try
        {
            reactor->trackMCTPInterface(path, interfaceEntry->second);
        }
        catch (const std::invalid_argument& ex)
        {
            warning(
                "Cannot track MCTP interface defined at '{INVENTORY_PATH}': {EXCEPTION}",
                "INVENTORY_PATH", path, "EXCEPTION", ex);
        }
    }

    auto deviceEntry = exposed.find(mctpDeviceInterface);
    if (deviceEntry != exposed.end())
    {
        try
        {
            reactor->manageMCTPDevice(path, deviceEntry->second);
        }
        catch (const std::invalid_argument& ex)
        {
            warning(
                "Cannot manage MCTP device defined at '{INVENTORY_PATH}': {EXCEPTION}",
                "INVENTORY_PATH", path, "EXCEPTION", ex);
        }
    }
}

static void removeInventory(const std::shared_ptr<MCTPReactor>& reactor,
                            sdbusplus::message_t& msg)
{
    auto [path, removed] =
        msg.unpack<sdbusplus::message::object_path, std::set<std::string>>();
    if (removed.contains(mctpDeviceInterface))
    {
        reactor->unmanageMCTPDevice(path.str);
    }

    if (removed.contains(mctpInterfaceInterface))
    {
        reactor->untrackMCTPInterface(path.str);
    }
}

static void manageMCTPEntity(const std::shared_ptr<MCTPReactor>& reactor,
                             ManagedObjectType& entities)
{
    for (const auto& [path, config] : entities)
    {
        auto interfaceEntry = config.find(mctpInterfaceInterface);
        if (interfaceEntry != config.end())
        {
            reactor->trackMCTPInterface(path, interfaceEntry->second);
        }

        auto deviceEntry = config.find(mctpDeviceInterface);
        if (deviceEntry != config.end())
        {
            reactor->manageMCTPDevice(path, deviceEntry->second);
        }
    }
}

static auto createMCTPDDevice(
    const std::shared_ptr<sdbusplus::asio::connection>& connection,
    const std::string& interface, const std::vector<std::uint8_t>& physaddr,
    std::optional<std::uint8_t> eid)
{
    return std::make_shared<MCTPDDevice>(connection, interface, physaddr, eid);
}

int main()
{
    constexpr std::chrono::seconds period(5);

    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    DBusAssociationServer associationServer(systemBus);
    auto createMCTPDevice = std::bind_front(createMCTPDDevice, systemBus);
    auto reactor = std::make_shared<MCTPReactor>(std::move(createMCTPDevice),
                                                 associationServer);
    boost::asio::steady_timer clock(io);

    std::function<void(const boost::system::error_code&)> alarm =
        [&](const boost::system::error_code& ec) {
        if (ec)
        {
            return;
        }
        clock.expires_after(period);
        clock.async_wait(alarm);
        reactor->tick();
    };
    clock.expires_after(period);
    clock.async_wait(alarm);

    systemBus->request_name("xyz.openbmc_project.MCTPReactor");

    using namespace sdbusplus::bus::match;

    const std::string interfacesRemovedMatchSpec =
        rules::sender("xyz.openbmc_project.EntityManager") +
        // Trailing slash on path: Listen for signals on the inventory subtree
        rules::interfacesRemovedAtPath("/xyz/openbmc_project/inventory/");

    auto interfacesRemovedMatch = sdbusplus::bus::match_t(
        static_cast<sdbusplus::bus_t&>(*systemBus), interfacesRemovedMatchSpec,
        std::bind_front(removeInventory, reactor));

    const std::string interfacesAddedMatchSpec =
        rules::sender("xyz.openbmc_project.EntityManager") +
        // Trailing slash on path: Listen for signals on the inventory subtree
        rules::interfacesAddedAtPath("/xyz/openbmc_project/inventory/");

    auto interfacesAddedMatch = sdbusplus::bus::match_t(
        static_cast<sdbusplus::bus_t&>(*systemBus), interfacesAddedMatchSpec,
        std::bind_front(addInventory, reactor));

    boost::asio::post(io, [reactor, systemBus]() {
        auto gsc = std::make_shared<GetSensorConfiguration>(
            systemBus, std::bind_front(manageMCTPEntity, reactor));
        gsc->getConfiguration({"MCTPInterface", "MCTPDevice"});
    });

    io.run();

    return EXIT_SUCCESS;
}
