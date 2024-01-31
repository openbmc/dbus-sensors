#include "MCTPReactor.hpp"

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

void MCTPReactor::deferSetup(const std::shared_ptr<MCTPDevice>& dev)
{
    info("Deferring setup for MCTP device at [ {MCTP_DEVICE} ]", "MCTP_DEVICE",
         dev->describe());

    deferred.emplace(dev);
}

void MCTPReactor::untrackEndpoint(const std::shared_ptr<MCTPEndpoint>& ep)
{
    server.disassociate(MCTPDEndpoint::path(ep));
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
    // # busctl call xyz.openbmc_project.ObjectMapper /xyz/openbmc_project/object_mapper xyz.openbmc_project.ObjectMapper GetAssociatedSubTree ooias /xyz/openbmc_project/mctp/1/9/configured_by / 0 1 xyz.openbmc_project.Configuration.MCTPDevice
    // a{sa{sas}} 1 "/xyz/openbmc_project/inventory/system/nvme/NVMe_1/NVMe_1_Temp" 1 "xyz.openbmc_project.EntityManager" 1 "xyz.openbmc_project.Configuration.MCTPDevice"
    // ```
    // clang-format on
    const std::string& item = devices.inventoryFor(ep->device());
    std::vector<Association> associations{
        {"configured_by", "configures", item}};
    server.associate(MCTPDEndpoint::path(ep), associations);
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

void MCTPReactor::tick()
{
    auto toSetup = std::exchange(deferred, {});
    for (const auto& entry : toSetup)
    {
        setupEndpoint(entry);
    }
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

    auto device = createDevice(interface, encoded, eid);
    devices.add(path, device);

    info("Starting management of MCTP device at [ {MCTP_DEVICE} ]",
         "MCTP_DEVICE", device->describe());

    setupEndpoint(device);
}

void MCTPReactor::unmanageMCTPDevice(const std::string& path)
{
    if (!devices.contains(path))
    {
        return;
    }

    std::shared_ptr<MCTPDevice> device = devices.deviceFor(path);

    info("MCTP device inventory removed at '{INVENTORY_PATH}'",
         "INVENTORY_PATH", path);

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
        throw std::invalid_argument(
            "Provided object violates MCTPInterface schema");
    }

    auto name = std::visit(VariantToStringVisitor(), mName->second);
    auto transport = std::visit(VariantToStringVisitor(), mTransport->second);
    if (transport != "SMBus")
    {
        throw std::invalid_argument(
            std::format("Unsupported MCTP transport: {}", transport));
    }

    interfaceConfiguration.emplace(std::pair<std::string, MCTPInterface>(
        name, {name, MCTPTransport::SMBus}));
    interfaceInventory.emplace(path, name);

    info("Tracking MCTP interface '{MCTP_INTERFACE}' ({MCTP_TRANSPORT})",
         "MCTP_INTERFACE", name, "MCTP_TRANSPORT", transport);

    // XXX: Figure out a more effective data structure?
    auto candidates = std::exchange(unreachable, {});
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
