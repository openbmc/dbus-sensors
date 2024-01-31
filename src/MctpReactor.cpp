#include "MctpEndpoint.hpp"
#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/system/detail/error_code.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <system_error>

PHOSPHOR_LOG2_USING;

static constexpr const char* const mctpEndpointInterface =
    "xyz.openbmc_project.Configuration.MCTPEndpoint";

class MctpDeviceRepository
{
  public:
    MctpDeviceRepository() = default;
    MctpDeviceRepository(const MctpDeviceRepository&) = delete;
    MctpDeviceRepository(MctpDeviceRepository&&) = delete;
    ~MctpDeviceRepository() = default;

    MctpDeviceRepository& operator=(const MctpDeviceRepository&) = delete;
    MctpDeviceRepository& operator=(MctpDeviceRepository&&) = delete;

    void add(const std::string& inventory,
             const std::shared_ptr<MctpDevice>& device);
    void remove(const std::shared_ptr<MctpDevice>& device);
    void remove(const std::string& inventory);
    bool contains(const std::shared_ptr<MctpDevice>& device);
    const std::string& inventoryFor(const std::shared_ptr<MctpDevice>& device);
    const std::shared_ptr<MctpDevice>& deviceFor(const std::string& inventory);

  private:
    // FIXME: Ugh, hack. Figure out a better data structure?
    std::map<std::string, std::shared_ptr<MctpDevice>> devices;

    auto lookup(const std::shared_ptr<MctpDevice>& device);
};

void MctpDeviceRepository::add(const std::string& inventory,
                               const std::shared_ptr<MctpDevice>& device)
{
    auto [_, fresh] = devices.emplace(inventory, device);
    if (!fresh)
    {
        throw std::logic_error(std::format(
            "Tried to add entry for existing device: {}", device->describe()));
    }
}

auto MctpDeviceRepository::lookup(const std::shared_ptr<MctpDevice>& device)
{
    return std::find_if(devices.begin(), devices.end(),
                        [needle{device.get()}](const auto& it) {
        return it.second.get() == needle;
    });
}

void MctpDeviceRepository::remove(const std::shared_ptr<MctpDevice>& device)
{
    auto entry = lookup(device);
    if (entry == devices.end())
    {
        throw std::logic_error(std::format(
            "Trying to remove unknown device: {}", entry->second->describe()));
    }
    devices.erase(entry);
}

void MctpDeviceRepository::remove(const std::string& inventory)
{
    auto entry = devices.find(inventory);
    if (entry == devices.end())
    {
        throw std::logic_error(
            std::format("Trying to remove unknown inventory: {}", inventory));
    }
    devices.erase(entry);
}

bool MctpDeviceRepository::contains(const std::shared_ptr<MctpDevice>& device)
{
    return lookup(device) != devices.end();
}

const std::string& MctpDeviceRepository::inventoryFor(
    const std::shared_ptr<MctpDevice>& device)
{
    auto entry = lookup(device);
    if (entry == devices.end())
    {
        throw std::logic_error(
            std::format("Cannot retrieve inventory for unknown device: {}",
                        device->describe()));
    }
    return entry->first;
}

const std::shared_ptr<MctpDevice>&
    MctpDeviceRepository::deviceFor(const std::string& inventory)
{
    auto entry = devices.find(inventory);
    if (entry == devices.end())
    {
        throw std::logic_error(std::format(
            "Cannot retrieve device for unknown inventory: {}", inventory));
    }
    return entry->second;
}

class MctpReactor : public std::enable_shared_from_this<MctpReactor>
{
  public:
    MctpReactor() = delete;
    MctpReactor(const MctpReactor&) = delete;
    MctpReactor(MctpReactor&&) = delete;
    MctpReactor(boost::asio::io_context& io,
                const std::shared_ptr<sdbusplus::asio::connection>& conn);
    ~MctpReactor() = default;
    MctpReactor& operator=(const MctpReactor&) = delete;
    MctpReactor& operator=(MctpReactor&&) = delete;

    void manageMctpEndpoint(const std::string& configPath,
                            const SensorData& config);
    void unmanageMctpEndpoint(const std::string& path);

  private:
    std::shared_ptr<sdbusplus::asio::connection> connection;
    sdbusplus::asio::object_server server;
    MctpDeviceRepository devices;
    std::set<std::shared_ptr<MctpDevice>> deferred;
    const std::chrono::seconds period;
    boost::asio::steady_timer deferredTimer;
    std::map<std::string, std::shared_ptr<sdbusplus::asio::dbus_interface>>
        proxies;

    void setupDeferred();
    void deferSetup(const std::shared_ptr<MctpDevice>& dev);
    void setupEndpoint(const std::shared_ptr<MctpDevice>& dev);
    void trackEndpoint(const std::shared_ptr<MctpEndpoint>& ep);
    void untrackEndpoint(const std::shared_ptr<MctpEndpoint>& ep);
};

MctpReactor::MctpReactor(
    boost::asio::io_context& io,
    const std::shared_ptr<sdbusplus::asio::connection>& conn) :
    connection(conn),
    server(connection), period(5), deferredTimer(io)
{
    server.add_manager("/xyz/openbmc_project/mctp");
}

void MctpReactor::setupDeferred()
{
    auto toSetup(deferred);

    deferred.clear();

    for (const auto& entry : toSetup)
    {
        setupEndpoint(entry);
    }
}

void MctpReactor::deferSetup(const std::shared_ptr<MctpDevice>& dev)
{
    info("Deferring setup for MCTP device at [{MCTP_DEVICE}]", "MCTP_DEVICE",
         dev->describe());

    deferred.emplace(dev);

    if (deferred.size() > 1)
    {
        return;
    }

    deferredTimer.expires_after(period);
    deferredTimer.async_wait(
        [weak{weak_from_this()}](const boost::system::error_code& ec) {
        if (ec)
        {
            return;
        }

        if (auto self = weak.lock())
        {
            if (!self->deferred.empty())
            {
                self->setupDeferred();
            }
        }
    });
}

void MctpReactor::untrackEndpoint(const std::shared_ptr<MctpEndpoint>& ep)
{
    const std::string epPath = MctpdEndpoint::path(ep);
    const auto entry = proxies.find(epPath);
    if (entry == proxies.end())
    {
        throw std::logic_error(std::format(
            "Attempted to untrack path that was not tracked: {}", epPath));
    }
    std::shared_ptr<sdbusplus::asio::dbus_interface> iface = entry->second;
    server.remove_interface(entry->second);
    proxies.erase(entry);
}

void MctpReactor::trackEndpoint(const std::shared_ptr<MctpEndpoint>& ep)
{
    info("Device endpoint configured at [{MCTP_ENDPOINT}]", "MCTP_ENDPOINT",
         ep->describe());

    ep->subscribe(
        // Degraded
        [](const std::shared_ptr<MctpEndpoint>&) {},
        // Available
        [](const std::shared_ptr<MctpEndpoint>&) {},
        // Removed
        [weak{weak_from_this()}](const std::shared_ptr<MctpEndpoint>& ep) {
        info("Removed endpoint from MCTP device at [{MCTP_ENDPOINT}]",
             "MCTP_ENDPOINT", ep->describe());
        if (auto self = weak.lock())
        {
            self->untrackEndpoint(ep);
            // Only defer the setup if we know inventory is still present
            if (self->devices.contains(ep->device()))
            {
                self->deferSetup(ep->device());
            }
        }
    });

    // XXX: Proxy-host the association back to the inventory at the same path as
    // the endpoint in mctpd.
    // TODO: Evaluate how much of a hack this is and whether we should do
    // something better.
    const auto endpointPath = MctpdEndpoint::path(ep);
    auto [entry, _] = proxies.emplace(
        endpointPath,
        server.add_interface(endpointPath, association::interface));
    std::shared_ptr<sdbusplus::asio::dbus_interface> iface = entry->second;
    const std::string& item = devices.inventoryFor(ep->device());
    std::vector<Association> associations{{"chassis", "mctp_endpoints", item}};
    iface->register_property("Associations", associations);
    iface->initialize();
}

void MctpReactor::setupEndpoint(const std::shared_ptr<MctpDevice>& dev)
{
    info("Attempting to setup up MCTP endpoint for device at [{MCTP_DEVICE}]",
         "MCTP_DEVICE", dev->describe());
    dev->setup([weak{weak_from_this()},
                dev](const std::error_code& ec,
                     const std::shared_ptr<MctpEndpoint>& ep) mutable {
        auto self = weak.lock();
        if (!self)
        {
            return;
        }

        if (ec)
        {
            error(
                "Setup failed for MCTP device at [{MCTP_DEVICE}], deferring: {ERROR_MESSAGE}",
                "MCTP_DEVICE", dev->describe(), "ERROR_MESSAGE", ec.message());

            self->deferSetup(dev);
            return;
        }

        self->trackEndpoint(ep);
    });
}

void MctpReactor::manageMctpEndpoint(const std::string& configPath,
                                     const SensorData& config)
{
    auto endpointEntry = config.find(mctpEndpointInterface);
    if (endpointEntry == config.end())
    {
        return;
    }

    info("MCTP device inventory added at '{INVENTORY_PATH}'", "INVENTORY_PATH",
         configPath);

    auto iface = endpointEntry->second;

    auto mAddress = iface.find("Address");
    auto mBus = iface.find("Bus");
    if (mAddress == iface.end() || mBus == iface.end())
    {
        error(
            "Configuration object at '{INVENTORY_PATH}' violates the MCTPEndpoint schema",
            "INVENTORY_PATH", configPath);
        return;
    }

    auto address = std::visit(VariantToIntVisitor(), mAddress->second);
    auto bus = std::visit(VariantToIntVisitor(), mBus->second);

    if (address < 0 || address > UINT8_MAX)
    {
        error(
            "Configuration object at '{INVENTORY_PATH}' provided an invalid SMBus device address: {SMBUS_ADDRESS}",
            "INVENTORY_PATH", configPath, "SMBUS_ADDRESS", address);
        return;
    }

    if (bus < 0)
    {
        error(
            "Configuration object at '{INVENTORY_PATH}' provided an invalid SMBus bus index: {SMBUS_INDEX}",
            "INVENTORY_PATH", configPath, "SMBUS_INDEX", bus);
        return;
    }

    auto mEndpointId = iface.find("EndpointId");
    std::optional<std::uint8_t> eid{};
    if (mEndpointId != iface.end())
    {
        int eidv = std::visit(VariantToIntVisitor(), mEndpointId->second);
        if (eidv >= 0 || eidv <= UINT8_MAX)
        {
            eid = eidv;
        }
    }

    auto device = std::make_shared<SmbusMctpdDevice>(
        connection, bus, static_cast<uint8_t>(address), eid);
    devices.add(configPath, device);

    info("Starting management of MCTP device at [{MCTP_DEVICE}]", "MCTP_DEVICE",
         device->describe());

    setupEndpoint(device);
}

void MctpReactor::unmanageMctpEndpoint(const std::string& path)
{
    std::shared_ptr<MctpDevice> device = devices.deviceFor(path);

    info("MCTP device inventory removed at '{INVENTORY_PATH}'",
         "INVENTORY_PATH", path);

    auto proxyEntry = proxies.find(path);
    if (proxyEntry != proxies.end())
    {
        proxies.erase(proxyEntry);
    }

    deferred.erase(device);

    // Remove the device from the repository before notifying the device itself
    // of removal so we don't defer its setup
    devices.remove(device);

    info("Stopping management of MCTP device at [{MCTP_DEVICE}]", "MCTP_DEVICE",
         device->describe());

    device->remove();
}

static void addInventory(const std::shared_ptr<MctpReactor>& reactor,
                         sdbusplus::message_t& msg)
{
    sdbusplus::message::object_path op;
    SensorData exposed;

    msg.read(op);
    msg.read(exposed);

    reactor->manageMctpEndpoint(op, exposed);
}

static void removeInventory(const std::shared_ptr<MctpReactor>& reactor,
                            sdbusplus::message_t& msg)
{
    sdbusplus::message::object_path op;
    std::vector<std::string> removed;

    msg.read(op);
    msg.read(removed);

    if (std::find(removed.begin(), removed.end(), mctpEndpointInterface) ==
        removed.end())
    {
        return;
    }

    reactor->unmanageMctpEndpoint(op.str);
}

static void manageEndpoints(const std::shared_ptr<MctpReactor>& reactor,
                            ManagedObjectType& entities)
{
    for (const auto& [path, config] : entities)
    {
        reactor->manageMctpEndpoint(path, config);
    }
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    auto reactor = std::make_shared<MctpReactor>(io, systemBus);

    systemBus->request_name("xyz.openbmc_project.MctpReactor");

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
            systemBus, std::bind_front(manageEndpoints, reactor));
        gsc->getConfiguration({"MCTPEndpoint"});
    });

    io.run();

    return EXIT_SUCCESS;
}
