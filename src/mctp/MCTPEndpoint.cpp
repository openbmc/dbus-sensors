#include "MCTPEndpoint.hpp"

#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <bits/fs_dir.h>

#include <boost/system/detail/errc.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/exception.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cassert>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <format>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

PHOSPHOR_LOG2_USING;

static constexpr const char* mctpdBusName = "au.com.codeconstruct.MCTP1";
static constexpr const char* mctpdControlPath = "/au/com/codeconstruct/mctp1";
static constexpr const char* mctpdControlInterface =
    "au.com.codeconstruct.MCTP.BusOwner1";
static constexpr const char* mctpdEndpointControlInterface =
    "au.com.codeconstruct.MCTP.Endpoint1";

MCTPDDevice::MCTPDDevice(
    const std::shared_ptr<sdbusplus::asio::connection>& connection,
    const std::string& interface, const std::vector<uint8_t>& physaddr) :
    connection(connection), interface(interface), physaddr(physaddr)
{}

void MCTPDDevice::onEndpointInterfacesRemoved(
    const std::weak_ptr<MCTPDDevice>& weak, const std::string& objpath,
    sdbusplus::message_t& msg)
{
    auto path = msg.unpack<sdbusplus::message::object_path>();
    assert(path.str == objpath);

    auto removedIfaces = msg.unpack<std::set<std::string>>();
    if (!removedIfaces.contains(mctpdEndpointControlInterface))
    {
        return;
    }

    if (auto self = weak.lock())
    {
        self->endpointRemoved();
    }
    else
    {
        info(
            "Device for inventory at '{INVENTORY_PATH}' was destroyed concurrent to endpoint removal",
            "INVENTORY_PATH", objpath);
    }
}

void MCTPDDevice::finaliseEndpoint(
    const std::string& objpath, uint8_t eid, int network,
    std::function<void(const std::error_code& ec,
                       const std::shared_ptr<MCTPEndpoint>& ep)>& added)
{
    const auto matchSpec =
        sdbusplus::bus::match::rules::interfacesRemovedAtPath(objpath);
    removeMatch = std::make_unique<sdbusplus::bus::match_t>(
        *connection, matchSpec,
        std::bind_front(MCTPDDevice::onEndpointInterfacesRemoved,
                        weak_from_this(), objpath));
    endpoint = std::make_shared<MCTPDEndpoint>(shared_from_this(), connection,
                                               objpath, network, eid);
    added({}, endpoint);
}

void MCTPDDevice::setup(
    std::function<void(const std::error_code& ec,
                       const std::shared_ptr<MCTPEndpoint>& ep)>&& added)
{
    // Use a lambda to separate state validation from business logic,
    // where the business logic for a successful setup() is encoded in
    // MctpdDevice::finaliseEndpoint()
    auto onSetup = [weak{weak_from_this()}, added{std::move(added)}](
                       const boost::system::error_code& ec, uint8_t eid,
                       int network, const std::string& objpath,
                       bool allocated [[maybe_unused]]) mutable {
        if (ec)
        {
            added(ec, {});
            return;
        }

        if (auto self = weak.lock())
        {
            self->finaliseEndpoint(objpath, eid, network, added);
        }
        else
        {
            info(
                "Device object for inventory at '{INVENTORY_PATH}' was destroyed concurrent to completion of its endpoint setup",
                "INVENTORY_PATH", objpath);
        }
    };
    connection->async_method_call(
        onSetup, mctpdBusName,
        mctpdControlPath + std::string("/interfaces/") + interface,
        mctpdControlInterface, "AssignEndpoint", physaddr);
}

void MCTPDDevice::endpointRemoved()
{
    if (endpoint)
    {
        debug("Endpoint removed @ [ {MCTP_ENDPOINT} ]", "MCTP_ENDPOINT",
              endpoint->describe());
        removeMatch.reset();
        endpoint->removed();
        endpoint.reset();
    }
}

void MCTPDDevice::remove()
{
    if (endpoint)
    {
        debug("Removing endpoint @ [ {MCTP_ENDPOINT} ]", "MCTP_ENDPOINT",
              endpoint->describe());
        endpoint->remove();
    }
}

std::string MCTPDDevice::describe() const
{
    std::string description = std::format("interface: {}", interface);
    if (!physaddr.empty())
    {
        description.append(", address: 0x [ ");
        auto it = physaddr.begin();
        for (; it != physaddr.end() - 1; it++)
        {
            description.append(std::format("{:02x} ", *it));
        }
        description.append(std::format("{:02x} ]", *it));
    }
    return description;
}

// https://en.cppreference.com/w/cpp/utility/hash/operator().html
//
// https://en.wikipedia.org/w/index.php?title=Fowler%E2%80%93Noll%E2%80%93Vo_hash_function&oldid=1312413750#FNV_hash_parameters
template <typename T, typename V, T p>
static std::size_t fnv1a(T h, V e);

template <std::size_t S>
static std::size_t fnv1a(const std::vector<std::uint8_t>& d);

template <typename V>
static std::size_t fnv1a(std::uint32_t h, V v)
{
    constexpr std::uint32_t p = 0x01000193;
    return (h ^ v) * p;
}

template <>
[[maybe_unused]] std::size_t fnv1a<4UL>(const std::vector<std::uint8_t>& d)
{
    std::uint32_t h = 0x811c9dc5;
    for (const auto& v : d)
    {
        h = fnv1a(h, v);
    }
    return h;
}

template <typename V>
static std::size_t fnv1a(std::uint64_t h, V v)
{
    constexpr std::uint64_t p = 0x00000100000001b3;
    return (h ^ v) * p;
}

template <>
[[maybe_unused]] std::size_t fnv1a<8UL>(const std::vector<std::uint8_t>& d)
{
    std::uint64_t h = 0xcbf29ce484222325;
    for (const auto& v : d)
    {
        h = fnv1a(h, v);
    }
    return h;
}

static std::size_t fnv1aHash(const std::vector<std::uint8_t>& d)
{
    return fnv1a<sizeof(std::size_t)>(d);
}

std::size_t MCTPDDevice::id() const
{
    std::size_t h1 = std::hash<std::string>{}(interface);
    std::size_t h2 = fnv1aHash(physaddr);

    return h1 ^ (h2 << 1);
}

std::string MCTPDEndpoint::path(const std::shared_ptr<MCTPEndpoint>& ep)
{
    return std::format("{}/networks/{}/endpoints/{}", mctpdControlPath,
                       ep->network(), ep->eid());
}

void MCTPDEndpoint::onMctpEndpointChange(sdbusplus::message_t& msg)
{
    auto [iface, changed, _] =
        msg.unpack<std::string, std::map<std::string, BasicVariantType>,
                   std::vector<std::string>>();
    if (iface != mctpdEndpointControlInterface)
    {
        return;
    }

    auto it = changed.find("Connectivity");
    if (it == changed.end())
    {
        return;
    }

    updateEndpointConnectivity(std::get<std::string>(it->second));
}

void MCTPDEndpoint::updateEndpointConnectivity(const std::string& connectivity)
{
    if (connectivity == "Degraded")
    {
        if (notifyDegraded)
        {
            notifyDegraded(shared_from_this());
        }
    }
    else if (connectivity == "Available")
    {
        if (notifyAvailable)
        {
            notifyAvailable(shared_from_this());
        }
    }
    else
    {
        debug("Unrecognised connectivity state: '{CONNECTIVITY_STATE}'",
              "CONNECTIVITY_STATE", connectivity);
    }
}

int MCTPDEndpoint::network() const
{
    return mctp.network;
}

uint8_t MCTPDEndpoint::eid() const
{
    return mctp.eid;
}

void MCTPDEndpoint::subscribe(Event&& degraded, Event&& available,
                              Event&& removed)
{
    const auto matchSpec =
        sdbusplus::bus::match::rules::propertiesChangedNamespace(
            objpath.str, mctpdEndpointControlInterface);

    this->notifyDegraded = std::move(degraded);
    this->notifyAvailable = std::move(available);
    this->notifyRemoved = std::move(removed);

    try
    {
        connectivityMatch.emplace(
            static_cast<sdbusplus::bus_t&>(*connection), matchSpec,
            [weak{weak_from_this()},
             path{objpath.str}](sdbusplus::message_t& msg) {
                if (auto self = weak.lock())
                {
                    self->onMctpEndpointChange(msg);
                }
                else
                {
                    info(
                        "The endpoint for the device at inventory path '{INVENTORY_PATH}' was destroyed concurrent to the removal of its state change match",
                        "INVENTORY_PATH", path);
                }
            });
        connection->async_method_call(
            [weak{weak_from_this()},
             path{objpath.str}](const boost::system::error_code& ec,
                                const std::variant<std::string>& value) {
                if (ec)
                {
                    debug(
                        "Failed to get current connectivity state: {ERROR_MESSAGE}",
                        "ERROR_MESSAGE", ec.message(), "ERROR_CATEGORY",
                        ec.category().name(), "ERROR_CODE", ec.value());
                    return;
                }

                if (auto self = weak.lock())
                {
                    const std::string& connectivity =
                        std::get<std::string>(value);
                    self->updateEndpointConnectivity(connectivity);
                }
                else
                {
                    info(
                        "The endpoint for the device at inventory path '{INVENTORY_PATH}' was destroyed concurrent to the completion of its connectivity state query",
                        "INVENTORY_PATH", path);
                }
            },
            mctpdBusName, objpath.str, "org.freedesktop.DBus.Properties", "Get",
            mctpdEndpointControlInterface, "Connectivity");
    }
    catch (const sdbusplus::exception::SdBusError& err)
    {
        this->notifyDegraded = nullptr;
        this->notifyAvailable = nullptr;
        this->notifyRemoved = nullptr;
        std::throw_with_nested(
            MCTPException("Failed to register connectivity signal match"));
    }
}

void MCTPDEndpoint::remove()
{
    connection->async_method_call(
        [self{shared_from_this()}](const boost::system::error_code& ec) {
            if (ec)
            {
                debug("Failed to remove endpoint @ [ {MCTP_ENDPOINT} ]",
                      "MCTP_ENDPOINT", self->describe());
                return;
            }
        },
        mctpdBusName, objpath.str, mctpdEndpointControlInterface, "Remove");
}

void MCTPDEndpoint::removed()
{
    if (notifyRemoved)
    {
        notifyRemoved(shared_from_this());
    }
}

std::string MCTPDEndpoint::describe() const
{
    return std::format("network: {}, EID: {} | {}", mctp.network, mctp.eid,
                       dev->describe());
}

std::shared_ptr<MCTPDevice> MCTPDEndpoint::device() const
{
    return dev;
}

std::optional<SensorBaseConfigMap> I2CMCTPDDevice::match(
    const SensorData& config)
{
    auto iface = config.find(configInterfaceName(configType));
    if (iface == config.end())
    {
        return std::nullopt;
    }
    return iface->second;
}

std::optional<SensorBaseConfigMap> I3CMCTPDDevice::match(
    const SensorData& config)
{
    auto iface = config.find(configInterfaceName(configType));
    if (iface == config.end())
    {
        return std::nullopt;
    }
    return iface->second;
}

bool I2CMCTPDDevice::match(const std::set<std::string>& interfaces)
{
    return interfaces.contains(configInterfaceName(configType));
}

bool I3CMCTPDDevice::match(const std::set<std::string>& interfaces)
{
    return interfaces.contains(configInterfaceName(configType));
}

std::shared_ptr<I2CMCTPDDevice> I2CMCTPDDevice::from(
    const std::shared_ptr<sdbusplus::asio::connection>& connection,
    const SensorBaseConfigMap& iface)
{
    auto mType = iface.find("Type");
    if (mType == iface.end())
    {
        throw std::invalid_argument(
            "No 'Type' member found for provided configuration object");
    }

    auto type = std::visit(VariantToStringVisitor(), mType->second);
    if (type != configType)
    {
        throw std::invalid_argument("Not an SMBus device");
    }

    auto mAddress = iface.find("Address");
    auto mBus = iface.find("Bus");
    auto mName = iface.find("Name");
    if (mAddress == iface.end() || mBus == iface.end() || mName == iface.end())
    {
        throw std::invalid_argument(
            "Configuration object violates MCTPI2CTarget schema");
    }

    auto sAddress = std::visit(VariantToStringVisitor(), mAddress->second);
    std::uint8_t address{};
    auto [aptr, aec] = std::from_chars(
        sAddress.data(), sAddress.data() + sAddress.size(), address);
    if (aec != std::errc{})
    {
        throw std::invalid_argument("Bad device address");
    }

    auto sBus = std::visit(VariantToStringVisitor(), mBus->second);
    int bus{};
    auto [bptr,
          bec] = std::from_chars(sBus.data(), sBus.data() + sBus.size(), bus);
    if (bec != std::errc{})
    {
        throw std::invalid_argument("Bad bus index");
    }

    try
    {
        return std::make_shared<I2CMCTPDDevice>(connection, bus, address);
    }
    catch (const MCTPException& ex)
    {
        warning(
            "Failed to create I2CMCTPDDevice at [ bus: {I2C_BUS}, address: {I2C_ADDRESS} ]: {EXCEPTION}",
            "I2C_BUS", bus, "I2C_ADDRESS", address, "EXCEPTION", ex);
        return {};
    }
}

std::shared_ptr<I3CMCTPDDevice> I3CMCTPDDevice::from(
    const std::shared_ptr<sdbusplus::asio::connection>& connection,
    const SensorBaseConfigMap& iface)
{
    auto mType = iface.find("Type");
    if (mType == iface.end())
    {
        throw std::invalid_argument(
            "No 'Type' member found for provided configuration object");
    }

    auto type = std::visit(VariantToStringVisitor(), mType->second);
    if (type != configType)
    {
        throw std::invalid_argument("Not an I3C device");
    }

    auto mAddress = iface.find("Address");
    auto mBus = iface.find("Bus");
    auto mName = iface.find("Name");
    if (mAddress == iface.end() || mBus == iface.end() || mName == iface.end())
    {
        throw std::invalid_argument(
            "Configuration object violates MCTPI3CTarget schema");
    }

    auto address = std::visit(VariantToNumArrayVisitor<uint8_t, uint64_t>(),
                              mAddress->second);
    if (address.empty())
    {
        throw std::invalid_argument("Bad device address");
    }

    auto sBus = std::visit(VariantToStringVisitor(), mBus->second);
    int bus{};
    auto [bptr,
          bec] = std::from_chars(sBus.data(), sBus.data() + sBus.size(), bus);
    if (bec != std::errc{})
    {
        throw std::invalid_argument("Bad bus index");
    }

    try
    {
        return std::make_shared<I3CMCTPDDevice>(connection, bus, address);
    }
    catch (const MCTPException& ex)
    {
        warning(
            "Failed to create I3CMCTPDDevice at [ bus: {I3C_BUS} ]: {EXCEPTION}",
            "I3C_BUS", bus, "EXCEPTION", ex);
        return {};
    }
}

std::string I2CMCTPDDevice::interfaceFromBus(int bus)
{
    std::filesystem::path netdir =
        std::format("/sys/bus/i2c/devices/i2c-{}/net", bus);
    std::error_code ec;
    std::filesystem::directory_iterator it(netdir, ec);
    if (ec || it == std::filesystem::end(it))
    {
        error("No net device associated with I2C bus {I2C_BUS} at {NET_DEVICE}",
              "I2C_BUS", bus, "NET_DEVICE", netdir);
        throw MCTPException("Bus is not configured as an MCTP interface");
    }

    return it->path().filename();
}

std::string I3CMCTPDDevice::interfaceFromBus(int bus)
{
    std::filesystem::path netdir = std::format("/sys/devices/virtual/net");
    std::error_code ec;
    std::filesystem::directory_iterator it(netdir, ec);
    if (ec || it == std::filesystem::end(it))
    {
        error("No net device associated with I3C bus {I3C_BUS} at {NET_DEVICE}",
              "I3C_BUS", bus, "NET_DEVICE", netdir);
        throw MCTPException("Bus is not configured as an MCTP interface");
    }

    std::string targetInterface = std::format("mctpi3c{}", bus);
    for (const auto& entry : std::filesystem::directory_iterator(netdir))
    {
        if (entry.is_directory() && entry.path().filename() == targetInterface)
        {
            return targetInterface;
        }
    }

    error("No matching net device found for I3C bus {I3C_BUS} at {NET_DEVICE}",
          "I3C_BUS", bus, "NET_DEVICE", netdir);
    throw MCTPException("No matching net device found for the specified bus");
}
