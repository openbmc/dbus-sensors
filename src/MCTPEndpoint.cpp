#include "MCTPEndpoint.hpp"

#include "Utils.hpp"

#include <boost/system/detail/errc.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/exception.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <exception>
#include <iterator>
#include <memory>
#include <sstream>
#include <system_error>

PHOSPHOR_LOG2_USING;

static constexpr const char* mctpdBusName = "xyz.openbmc_project.MCTP";
static constexpr const char* mctpdControlPath = "/xyz/openbmc_project/mctp";
static constexpr const char* mctpdControlInterface =
    "au.com.CodeConstruct.MCTP";
static constexpr const char* mctpdEndpointControlInterface =
    "au.com.CodeConstruct.MCTP.Endpoint";

MCTPDDevice::MCTPDDevice(
    const std::shared_ptr<sdbusplus::asio::connection>& connection,
    const std::string& interface, const std::vector<uint8_t>& physaddr,
    std::optional<std::uint8_t> eid) :
    connection(connection),
    interface(interface), physaddr(physaddr), eid{eid}
{}

void MCTPDDevice::onEndpointInterfacesRemoved(
    const std::weak_ptr<MCTPDDevice>& weak, const std::string& objpath,
    sdbusplus::message_t& msg)
{
    auto path = msg.unpack<sdbusplus::message::object_path>();
    if (path.str != objpath)
    {
        return;
    }

    auto removedIfaces = msg.unpack<std::set<std::string>>();
    if (!removedIfaces.contains(mctpdEndpointControlInterface))
    {
        return;
    }

    if (auto self = weak.lock())
    {
        self->endpointRemoved();
    }
}

void MCTPDDevice::finaliseEndpoint(
    const std::string& objpath, uint8_t eid, int network,
    std::function<void(const std::error_code& ec,
                       const std::shared_ptr<MCTPEndpoint>& ep)>&& added)
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
            self->finaliseEndpoint(objpath, eid, network, std::move(added));
        }
    };
    try
    {
        if (eid)
        {
            connection->async_method_call(
                onSetup, mctpdBusName, mctpdControlPath, mctpdControlInterface,
                "AssignEndpointStatic", interface, physaddr, *eid);
        }
        else
        {
            connection->async_method_call(
                onSetup, mctpdBusName, mctpdControlPath, mctpdControlInterface,
                "SetupEndpoint", interface, physaddr);
        }
    }
    catch (const sdbusplus::exception::SdBusError& err)
    {
        debug("Caught exception while configuring endpoint: {EXCEPTION}", "EXEPTION", err);
        auto errc = std::errc::no_such_device_or_address;
        auto ec = std::make_error_code(errc);
        added(ec, {});
    }
}

void MCTPDDevice::endpointRemoved()
{
    if (endpoint)
    {
        debug("Endpoint removed @ [ {MCTP_ENDPOINT} ]", "MCTP_ENDPOINT", endpoint->describe());
        removeMatch.reset();
        endpoint->removed();
        endpoint.reset();
    }
}

void MCTPDDevice::remove()
{
    if (endpoint)
    {
        debug("Removing endpoint @ [ {MCTP_ENDPOINT} ]", "MCTP_ENDPOINT", endpoint->describe());
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

std::string MCTPDEndpoint::path(const std::shared_ptr<MCTPEndpoint>& ep)
{
    return std::format("/xyz/openbmc_project/mctp/{}/{}", ep->network(),
                       ep->eid());
}

void MCTPDEndpoint::onMctpEndpointChange(sdbusplus::message_t& msg)
{
    auto [iface, changed, _] = msg.unpack<std::string, std::map<std::string, BasicVariantType>, std::vector<std::string>>();
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
        debug("Unrecognised connectivity state: '{CONNECTIVITY_STATE}'", "CONNECTIVITY_STATE", connectivity);
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

void MCTPDEndpoint::subscribe(Event&& degraded, Event&& available, Event&& removed)
{
    const auto matchSpec =
        sdbusplus::bus::match::rules::propertiesChangedNamespace(
            objpath.str, mctpdEndpointControlInterface);

    this->notifyDegraded = degraded;
    this->notifyAvailable = available;
    this->notifyRemoved = removed;

    try
    {
        connectivityMatch.emplace(
            static_cast<sdbusplus::bus_t&>(*connection), matchSpec,
            [weak{weak_from_this()}](sdbusplus::message_t& msg) {
            if (auto self = weak.lock())
            {
                self->onMctpEndpointChange(msg);
            }
        });
        connection->async_method_call(
            [weak{weak_from_this()}](const boost::system::error_code& ec,
                                     const std::variant<std::string>& value) {
            if (ec)
            {
                debug("Failed to get current connectivity state: {ERROR_MESSAGE}", "ERROR_MESSAGE", ec.message(), "ERROR_CATEGORY", ec.category().name(), "ERROR_CODE", ec.value());
                return;
            }

            if (auto self = weak.lock())
            {
                const std::string& connectivity = std::get<std::string>(value);
                self->updateEndpointConnectivity(connectivity);
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
    try
    {
        connection->async_method_call(
            [self{shared_from_this()}](const boost::system::error_code& ec) {
            if (ec)
            {
                debug("Failed to remove endpoint @ [ {MCTP_ENDPOINT} ]", "MCTP_ENDPOINT", self->describe());
                return;
            }
        },
            mctpdBusName, objpath.str, mctpdEndpointControlInterface, "Remove");
    }
    catch (const sdbusplus::exception::SdBusError& err)
    {
        std::throw_with_nested(
            MCTPException("Failed schedule endpoint removal"));
    }
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
    return std::format("network: {}, EID: {} | {}", mctp.network, mctp.eid, dev->describe());
}

std::shared_ptr<MCTPDevice> MCTPDEndpoint::device() const
{
    return dev;
}
