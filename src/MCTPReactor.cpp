#include "MCTPDeviceRepository.hpp"
#include "MCTPEndpoint.hpp"
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
#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <system_error>

PHOSPHOR_LOG2_USING;

static constexpr const char* const mctpInterfaceInterface =
    "xyz.openbmc_project.Configuration.MCTPInterface";

static constexpr const char* const mctpDeviceInterface =
    "xyz.openbmc_project.Configuration.MCTPDevice";

class MCTPReactor : public std::enable_shared_from_this<MCTPReactor>
{
  public:
    MCTPReactor() = delete;
    MCTPReactor(const MCTPReactor&) = delete;
    MCTPReactor(MCTPReactor&&) = delete;
    MCTPReactor(boost::asio::io_context& io,
                const std::shared_ptr<sdbusplus::asio::connection>& conn);
    ~MCTPReactor() = default;
    MCTPReactor& operator=(const MCTPReactor&) = delete;
    MCTPReactor& operator=(MCTPReactor&&) = delete;

    void manageMCTPDevice(const std::string& path,
                          const SensorBaseConfigMap& iface);
    void unmanageMCTPDevice(const std::string& path);
    void trackMCTPInterface(const std::string& path,
                            const SensorBaseConfigMap& iface);
    void untrackMCTPInterface(const std::string& path);

  private:
    std::shared_ptr<sdbusplus::asio::connection> connection;
    sdbusplus::asio::object_server server;

    // Maps the inventory DBus object path exposing an MCTP interface to an MCTP
    // interface name
    std::map<std::string, std::string> interfaceInventory;
    // Maps an MCTP interface name to the interface's properties
    std::map<std::string, MCTPInterface> interfaceConfiguration;

    MCTPDeviceRepository devices;

    // Maps the inventory DBus object path exposing an MCTP device to the
    // device's MCTP properties
    std::map<std::string, SensorBaseConfigMap> unreachable;

    // Tracks MCTP devices that have failed their setup
    std::set<std::shared_ptr<MCTPDevice>> deferred;

    const std::chrono::seconds period;
    boost::asio::steady_timer deferredTimer;
    std::map<std::string, std::shared_ptr<sdbusplus::asio::dbus_interface>>
        associations;

    void setupDeferred();
    void deferSetup(const std::shared_ptr<MCTPDevice>& dev);
    void setupEndpoint(const std::shared_ptr<MCTPDevice>& dev);
    void trackEndpoint(const std::shared_ptr<MCTPEndpoint>& ep);
    void untrackEndpoint(const std::shared_ptr<MCTPEndpoint>& ep);
};

MCTPReactor::MCTPReactor(
    boost::asio::io_context& io,
    const std::shared_ptr<sdbusplus::asio::connection>& conn) :
    connection(conn),
    server(connection), period(5), deferredTimer(io)
{
    server.add_manager("/xyz/openbmc_project/mctp");
}

void MCTPReactor::setupDeferred()
{
    auto toSetup = std::exchange(deferred, {});
    for (const auto& entry : toSetup)
    {
        setupEndpoint(entry);
    }
}

void MCTPReactor::deferSetup(const std::shared_ptr<MCTPDevice>& dev)
{
    info("Deferring setup for MCTP device at [ {MCTP_DEVICE} ]", "MCTP_DEVICE",
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

void MCTPReactor::untrackEndpoint(const std::shared_ptr<MCTPEndpoint>& ep)
{
    const std::string epPath = MCTPDEndpoint::path(ep);
    const auto entry = associations.find(epPath);
    if (entry == associations.end())
    {
        throw std::logic_error(std::format(
            "Attempted to untrack path that was not tracked: {}", epPath));
    }
    std::shared_ptr<sdbusplus::asio::dbus_interface> iface = entry->second;
    server.remove_interface(entry->second);
    associations.erase(entry);
}

void MCTPReactor::trackEndpoint(const std::shared_ptr<MCTPEndpoint>& ep)
{
    info("Device endpoint configured at [ {MCTP_ENDPOINT} ]", "MCTP_ENDPOINT",
         ep->describe());

    ep->subscribe(
        // Degraded
        [](const std::shared_ptr<MCTPEndpoint>&) {},
        // Available
        [](const std::shared_ptr<MCTPEndpoint>&) {},
        // Removed
        [weak{weak_from_this()}](const std::shared_ptr<MCTPEndpoint>& ep) {
        info("Removed endpoint from MCTP device at [ {MCTP_ENDPOINT} ]",
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

    // Proxy-host the association back to the inventory at the same path as the
    // endpoint in mctpd.
    //
    // clang-format off
    // ```
    // # busctl call xyz.openbmc_project.ObjectMapper /xyz/openbmc_project/object_mapper xyz.openbmc_project.ObjectMapper GetAssociatedSubTree ooias /xyz/openbmc_project/mctp/1/9/chassis / 0 1 xyz.openbmc_project.Configuration.MCTPDevice
    // a{sa{sas}} 1 "/xyz/openbmc_project/inventory/system/nvme/NVMe_1/NVMe_1_Temp" 1 "xyz.openbmc_project.EntityManager" 1 "xyz.openbmc_project.Configuration.MCTPDevice"
    // ```
    // clang-format on
    const auto endpointPath = MCTPDEndpoint::path(ep);
    auto [entry, _] = associations.emplace(
        endpointPath,
        server.add_interface(endpointPath, association::interface));
    std::shared_ptr<sdbusplus::asio::dbus_interface> iface = entry->second;
    const std::string& item = devices.inventoryFor(ep->device());
    std::vector<Association> associations{{"chassis", "mctp_endpoints", item}};
    iface->register_property("Associations", associations);
    iface->initialize();
}

void MCTPReactor::setupEndpoint(const std::shared_ptr<MCTPDevice>& dev)
{
    info("Attempting to setup up MCTP endpoint for device at [ {MCTP_DEVICE} ]",
         "MCTP_DEVICE", dev->describe());
    dev->setup([weak{weak_from_this()},
                dev](const std::error_code& ec,
                     const std::shared_ptr<MCTPEndpoint>& ep) mutable {
        auto self = weak.lock();
        if (!self)
        {
            return;
        }

        if (ec)
        {
            error(
                "Setup failed for MCTP device at [ {MCTP_DEVICE} ], deferring: {ERROR_MESSAGE}",
                "MCTP_DEVICE", dev->describe(), "ERROR_MESSAGE", ec.message());

            self->deferSetup(dev);
            return;
        }

        self->trackEndpoint(ep);
    });
}

static std::vector<uint8_t> encodeDeviceAddress(MCTPTransport transport,
                                                const std::string& address)
{
    if (transport == MCTPTransport::SMBus)
    {
        std::uint8_t res{};
        auto [_, ec] = std::from_chars(address.data(),
                                       address.data() + address.size(), res);
        if (ec == std::errc())
        {
            return {res};
        }
        error("Invalid address: {ERRC_DESCRIPTION}", "ERRC",
              static_cast<int>(ec), "ERRC_DESCRIPTION",
              std::make_error_code(ec).message());
    }

    return {};
}

void MCTPReactor::manageMCTPDevice(const std::string& path,
                                   const SensorBaseConfigMap& iface)
{
    auto mAddress = iface.find("Address");
    auto mInterface = iface.find("Interface");
    if (mAddress == iface.end() || mInterface == iface.end())
    {
        error(
            "Configuration object at '{INVENTORY_PATH}' violates the MCTPDevice schema",
            "INVENTORY_PATH", path);
        return;
    }

    auto interface = std::visit(VariantToStringVisitor(), mInterface->second);
    auto interfaceEntry = interfaceConfiguration.find(interface);
    if (interfaceEntry == interfaceConfiguration.end())
    {
        info(
            "Device at '{INVENTORY_PATH}' specified unconfigured MCTP interface '{MCTP_INTERFACE}', deferring setup",
            "INVENTORY_PATH", path, "MCTP_INTERFACE", interface);
        unreachable.emplace(path, iface);
        return;
    }

    MCTPTransport transport = interfaceEntry->second.transport;
    auto address = std::visit(VariantToStringVisitor(), mAddress->second);
    std::vector<std::uint8_t> encoded = encodeDeviceAddress(transport, address);
    if (encoded.empty())
    {
        error(
            "Unable to encode {MCTP_TRANSPORT} address '{MCTP_DEVICE_ADDRESS}' for device at '{INVENTORY_PATH}'",
            "MCTP_TRANSPORT", std::string("SMBus"), "MCTP_DEVICE_ADDRESS",
            address, "INVENTORY_PATH", path);
        return;
    }

    info("MCTP device inventory added at '{INVENTORY_PATH}'", "INVENTORY_PATH",
         path);

    auto mStaticEndpointId = iface.find("StaticEndpointID");
    std::optional<std::uint8_t> eid{};
    if (mStaticEndpointId != iface.end())
    {
        auto eids = std::visit(VariantToStringVisitor(),
                               mStaticEndpointId->second);
        std::uint8_t eidv{};
        auto [_, ec] = std::from_chars(eids.data(), eids.data() + eids.size(),
                                       eidv);
        if (ec == std::errc())
        {
            eid = eidv;
        }
        else
        {
            error("Invalid static endpoint ID: {ERROR_MESSAGE}", "ERROR_CODE",
                  static_cast<int>(ec), "ERROR_MESSAGE",
                  std::make_error_code(ec).message());
        }
    }

    auto device = std::make_shared<MCTPDDevice>(connection, interface, encoded,
                                                eid);
    devices.add(path, device);

    info("Starting management of MCTP device at [ {MCTP_DEVICE} ]",
         "MCTP_DEVICE", device->describe());

    setupEndpoint(device);
}

void MCTPReactor::unmanageMCTPDevice(const std::string& path)
{
    std::shared_ptr<MCTPDevice> device = devices.deviceFor(path);

    info("MCTP device inventory removed at '{INVENTORY_PATH}'",
         "INVENTORY_PATH", path);

    auto associationEntry = associations.find(path);
    if (associationEntry != associations.end())
    {
        associations.erase(associationEntry);
    }

    unreachable.erase(path);
    deferred.erase(device);

    // Remove the device from the repository before notifying the device itself
    // of removal so we don't defer its setup
    devices.remove(device);

    info("Stopping management of MCTP device at [ {MCTP_DEVICE} ]",
         "MCTP_DEVICE", device->describe());

    device->remove();
}

void MCTPReactor::trackMCTPInterface(const std::string& path,
                                     const SensorBaseConfigMap& iface)
{
    auto mName = iface.find("Name");
    auto mTransport = iface.find("Transport");

    if (mName == iface.end() || mTransport == iface.end())
    {
        error(
            "Configuration object at '{INVENTORY_PATH}' violates the MCTPInterface schema",
            "INVENTORY_PATH", path);
        return;
    }

    auto name = std::visit(VariantToStringVisitor(), mName->second);
    auto transport = std::visit(VariantToStringVisitor(), mTransport->second);
    if (transport != "SMBus")
    {
        warning(
            "Unsupported MCTP transport defined by configuration object at '{INVENTORY_PATH}': {MCTP_TRANSPORT}'",
            "INVENTORY_PATH", path, "MCTP_TRANSPORT", transport);
        return;
    }

    interfaceConfiguration.emplace(std::pair<std::string, MCTPInterface>(
        name, {name, MCTPTransport::SMBus}));
    interfaceInventory.emplace(path, name);

    info("Tracking MCTP interface '{MCTP_INTERFACE}' ({MCTP_TRANSPORT})",
         "MCTP_INTERFACE", name, "MCTP_TRANSPORT", transport);

    // XXX: Figure out a more effective data structure?
    auto candidates = unreachable;
    unreachable.clear();
    for (const auto& [path, config] : candidates)
    {
        manageMCTPDevice(path, config);
    }
}

void MCTPReactor::untrackMCTPInterface(const std::string& path)
{
    auto inventoryEntry = interfaceInventory.find(path);
    if (inventoryEntry == interfaceInventory.end())
    {
        return;
    }

    info("Untracking MCTP interface '{MCTP_INTERFACE}'", "MCTP_INTERFACE",
         inventoryEntry->second);

    interfaceConfiguration.erase(inventoryEntry->second);
    interfaceInventory.erase(inventoryEntry);
}

static void addInventory(const std::shared_ptr<MCTPReactor>& reactor,
                         sdbusplus::message_t& msg)
{
    sdbusplus::message::object_path path;
    SensorData exposed;

    msg.read(path);
    msg.read(exposed);

    auto interfaceEntry = exposed.find(mctpInterfaceInterface);
    if (interfaceEntry != exposed.end())
    {
        reactor->trackMCTPInterface(path, interfaceEntry->second);
    }

    auto deviceEntry = exposed.find(mctpDeviceInterface);
    if (deviceEntry != exposed.end())
    {
        reactor->manageMCTPDevice(path, deviceEntry->second);
    }
}

static void removeInventory(const std::shared_ptr<MCTPReactor>& reactor,
                            sdbusplus::message_t& msg)
{
    sdbusplus::message::object_path path;
    std::vector<std::string> removed;

    msg.read(path);
    msg.read(removed);

    auto deviceEntry = std::find(removed.begin(), removed.end(),
                                 mctpDeviceInterface);
    if (deviceEntry != removed.end())
    {
        reactor->unmanageMCTPDevice(path.str);
    }

    auto interfaceEntry = std::find(removed.begin(), removed.end(),
                                    mctpInterfaceInterface);
    if (interfaceEntry != removed.end())
    {
        reactor->untrackMCTPInterface(path.str);
    }
}

static void manageEndpoints(const std::shared_ptr<MCTPReactor>& reactor,
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

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    auto reactor = std::make_shared<MCTPReactor>(io, systemBus);

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
            systemBus, std::bind_front(manageEndpoints, reactor));
        gsc->getConfiguration({"MCTPInterface", "MCTPDevice"});
    });

    io.run();

    return EXIT_SUCCESS;
}
