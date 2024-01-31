#include "MctpEndpoint.hpp"
#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/system/detail/error_code.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <system_error>

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
    std::cout << "Deferring setup for device @ [" << dev->describe() << "]"
              << std::endl;

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
    std::cerr << "Interface at " << epPath << " has " << iface.use_count()
              << " references" << std::endl;
}

void MctpReactor::trackEndpoint(const std::shared_ptr<MctpEndpoint>& ep)
{
    std::cout << "Device endpoint configured @ [" << ep->describe() << "]"
              << std::endl;

    ep->subscribe(
        // Degraded
        [](const std::shared_ptr<MctpEndpoint>&) {},
        // Available
        [](const std::shared_ptr<MctpEndpoint>&) {},
        // Removed
        [weak{weak_from_this()}](const std::shared_ptr<MctpEndpoint>& ep) {
        std::cout << "Removed endpoint from device @ [" << ep->describe() << "]"
                  << std::endl;
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
    std::cout << "Attempting to set up MCTP endpoint for device @ ["
              << dev->describe() << "]" << std::endl;
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
            std::cerr << "Setup failed for device @ [" << dev->describe()
                      << "], deferring: " << ec.message() << std::endl;

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

    auto iface = endpointEntry->second;

    auto mAddress = iface.find("Address");
    auto mBus = iface.find("Bus");
    auto mEndpointId = iface.find("EndpointId");
    if (mAddress == iface.end() || mBus == iface.end() ||
        mEndpointId == iface.end())
    {
        std::cerr << "Config object violates MCTPEndpoint schema" << std::endl;
        return;
    }

    auto address = std::visit(VariantToIntVisitor(), mAddress->second);
    auto bus = std::visit(VariantToIntVisitor(), mBus->second);
    auto eid = std::visit(VariantToIntVisitor(), mEndpointId->second);

    if (address < 0 || address > UINT8_MAX || bus < 0 || eid < 0 ||
        eid > UINT8_MAX)
    {
        return;
    }

    // FIXME: Add support for static EIDs to SmbusMctpdDevice
    auto device = std::make_shared<SmbusMctpdDevice>(
        connection, bus, static_cast<uint8_t>(address));
    devices.add(configPath, device);
    std::cout << "Starting management of device @ [" << device->describe()
              << "]" << std::endl;
    setupEndpoint(device);
}

void MctpReactor::unmanageMctpEndpoint(const std::string& path)
{
    std::shared_ptr<MctpDevice> device = devices.deviceFor(path);
    std::cout << "Stopping management of device @ [" << device->describe()
              << "]" << std::endl;

    auto proxyEntry = proxies.find(path);
    if (proxyEntry != proxies.end())
    {
        proxies.erase(proxyEntry);
    }

    deferred.erase(device);
    // Remove the device from the repository before notifying the device itself
    // of removal so we don't defer its setup
    devices.remove(device);
    device->remove();
}

static void addInventory(const std::shared_ptr<MctpReactor>& reactor,
                         sdbusplus::message_t& msg)
{
    sdbusplus::message::object_path op;
    SensorData exposed;

    msg.read(op);
    msg.read(exposed);

    std::cout << "Signalled addition of inventory @ " << op.str << std::endl;

    reactor->manageMctpEndpoint(op, exposed);
}

static void removeInventory(const std::shared_ptr<MctpReactor>& reactor,
                            sdbusplus::message_t& msg)
{
    sdbusplus::message::object_path op;
    std::vector<std::string> removed;

    msg.read(op);
    msg.read(removed);

    std::cout << "Signalled removal of inventory @ " << op.str << std::endl;

    if (std::find(removed.begin(), removed.end(), mctpEndpointInterface) ==
        removed.end())
    {
        std::cerr << "Failed to find " << mctpEndpointInterface << std::endl;
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
