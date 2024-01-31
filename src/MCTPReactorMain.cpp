#include "MCTPEndpoint.hpp"
#include "MCTPReactor.hpp"
#include "Utils.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/message/native_types.hpp>

PHOSPHOR_LOG2_USING;

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

static std::shared_ptr<MCTPDevice> deviceFromConfig(
    const std::shared_ptr<sdbusplus::asio::connection>& connection,
    const SensorData& config)
{
    try
    {
        std::optional<SensorBaseConfigMap> iface;
        if ((iface = I2CMCTPDDevice::match(config)))
        {
            return I2CMCTPDDevice::from(connection, *iface);
        }
    }
    catch (const std::invalid_argument& ex)
    {
        error("Unable to create device: {EXCEPTION}", "EXCEPTION", ex);
    }

    return {};
}

static void
    addInventory(const std::shared_ptr<sdbusplus::asio::connection>& connection,
                 const std::shared_ptr<MCTPReactor>& reactor,
                 sdbusplus::message_t& msg)
{
    auto [path,
          exposed] = msg.unpack<sdbusplus::message::object_path, SensorData>();
    reactor->manageMCTPDevice(path, deviceFromConfig(connection, exposed));
}

static void removeInventory(const std::shared_ptr<MCTPReactor>& reactor,
                            sdbusplus::message_t& msg)
{
    auto [path, removed] =
        msg.unpack<sdbusplus::message::object_path, std::set<std::string>>();
    if (I2CMCTPDDevice::match(removed))
    {
        reactor->unmanageMCTPDevice(path.str);
    }
}

static void manageMCTPEntity(
    const std::shared_ptr<sdbusplus::asio::connection>& connection,
    const std::shared_ptr<MCTPReactor>& reactor, ManagedObjectType& entities)
{
    for (const auto& [path, config] : entities)
    {
        reactor->manageMCTPDevice(path, deviceFromConfig(connection, config));
    }
}

int main()
{
    constexpr std::chrono::seconds period(5);

    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    DBusAssociationServer associationServer(systemBus);
    auto reactor = std::make_shared<MCTPReactor>(associationServer);
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
        std::bind_front(addInventory, systemBus, reactor));

    boost::asio::post(io, [reactor, systemBus]() {
        auto gsc = std::make_shared<GetSensorConfiguration>(
            systemBus, std::bind_front(manageMCTPEntity, systemBus, reactor));
        gsc->getConfiguration({"MCTPI2CTarget"});
    });

    io.run();

    return EXIT_SUCCESS;
}
