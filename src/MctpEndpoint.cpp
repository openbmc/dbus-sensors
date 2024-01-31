#include "MctpEndpoint.hpp"

#include "Utils.hpp"

#include <boost/system/detail/errc.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/exception.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <exception>
#include <memory>
#include <system_error>

static constexpr const char* mctpdBusName = "xyz.openbmc_project.MCTP";
static constexpr const char* mctpdControlPath = "/xyz/openbmc_project/mctp";
static constexpr const char* mctpdControlInterface =
    "au.com.CodeConstruct.MCTP";
static constexpr const char* mctpdEndpointControlInterface =
    "au.com.CodeConstruct.MCTP.Endpoint";

MctpdDevice::MctpdDevice(
    const std::shared_ptr<sdbusplus::asio::connection>& connection,
    const std::string& interface, const std::vector<uint8_t>& physaddr) :
    connection(connection),
    interface(interface), physaddr(physaddr)
{}

void MctpdDevice::setup(
    std::function<void(const std::error_code& ec,
                       const std::shared_ptr<MctpEndpoint>& ep)>&& added)
{
    try
    {
        connection->async_method_call(
            [weak{weak_from_this()}, added{std::move(added)}](
                const boost::system::error_code& ec, uint8_t eid, int network,
                const std::string& objpath, bool allocated [[maybe_unused]]) {
            std::cerr << "Got method call response: " << ec.message()
                      << std::endl;
            if (ec)
            {
                /* XXX What error does mctpd actually provide? */
                added(ec, {});
                return;
            }

            if (auto self = weak.lock())
            {
                using namespace sdbusplus::bus::match;
                const auto matchSpec = rules::type::signal() +
                                       rules::member("InterfacesRemoved") +
                                       rules::argNpath(0, objpath);
                self->removeMatch = std::make_unique<sdbusplus::bus::match_t>(
                    *self->connection, matchSpec,
                    [weak{self->weak_from_this()},
                     objpath{objpath}](sdbusplus::message_t& msg) mutable {
                    sdbusplus::message::object_path path;
                    msg.read(path);

                    if (path.str != objpath)
                    {
                        return;
                    }

                    std::vector<std::string> removedIfaces;
                    msg.read(removedIfaces);

                    auto entry = std::find(removedIfaces.begin(),
                                           removedIfaces.end(),
                                           mctpdEndpointControlInterface);

                    if (entry == removedIfaces.end())
                    {
                        return;
                    }

                    if (auto self = weak.lock())
                    {
                        self->removeMatch.reset();
                        self->endpointRemoved();
                    }
                });
                self->endpoint = std::make_shared<MctpdEndpoint>(
                    self->connection, objpath, network, eid);
                added(static_cast<const std::error_code&>(ec), self->endpoint);
            }
        },
            mctpdBusName, mctpdControlPath, mctpdControlInterface,
            "SetupEndpoint", interface, physaddr);
    }
    catch (const sdbusplus::exception::SdBusError& err)
    {
        std::cerr << "Caught exception from SetupEndpoint invocation: "
                  << err.description() << std::endl;
        auto errc = std::errc::no_such_device_or_address;
        auto ec = std::make_error_code(errc);
        added(ec, {});
    }
}

void MctpdDevice::endpointRemoved()
{
    if (endpoint)
    {
        std::cerr << "Endpoint removed @ [" << endpoint->describe() << "]"
                  << std::endl;
        endpoint->removed();
        endpoint.reset();
    }
}

void MctpdDevice::remove()
{
    /* We're driving the removal - avoid triggering the match */
    removeMatch.reset();
    if (endpoint)
    {
        std::cerr << "Removing endpoint @ [" << endpoint->describe() << "]"
                  << std::endl;
        endpoint->remove();
        endpoint.reset();
    }
}

SmbusMctpdDevice::SmbusMctpdDevice(
    const std::shared_ptr<sdbusplus::asio::connection>& connection, int smbus,
    uint8_t smdev) :
    MctpdDevice(connection, std::string("mctpi2c") + std::to_string(smbus),
                {smdev}),
    smbus(smbus), smdev(smdev)
{}

std::string SmbusMctpdDevice::describe()
{
    return std::string("bus: ")
        .append(std::to_string(smbus))
        .append(", address: ")
        .append(std::to_string(smdev));
}

MctpdEndpoint::MctpdEndpoint(
    const std::shared_ptr<sdbusplus::asio::connection>& connection,
    sdbusplus::message::object_path objpath, int network, uint8_t eid) :
    connection(connection),
    objpath(std::move(objpath)), mctp{network, eid}
{}

void MctpdEndpoint::onMctpEndpointChange(sdbusplus::message_t& msg)
{
    std::string iface;
    std::map<std::string, BasicVariantType> changed;
    std::vector<std::string> invalidated;

    msg.read(iface);
    msg.read(changed);
    msg.read(invalidated);

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

void MctpdEndpoint::updateEndpointConnectivity(const std::string& connectivity)
{
    if (connectivity == "Degraded")
    {
        if (notifyDegraded)
        {
            notifyDegraded();
        }
    }
    else if (connectivity == "Available")
    {
        if (notifyAvailable)
        {
            notifyAvailable();
        }
    }
    else
    {
        std::cerr << "Unrecognised connectivity state: '" << connectivity << "'"
                  << std::endl;
    }
}

int MctpdEndpoint::network() const
{
    return mctp.network;
}

uint8_t MctpdEndpoint::eid() const
{
    return mctp.eid;
}

void MctpdEndpoint::subscribe(std::function<void()>&& degraded,
                              std::function<void()>&& available,
                              std::function<void()>&& removed)
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
                std::cerr << "Failed to get current connectivity state: " << ec
                          << std::endl;
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
            MctpException("Failed to register connectivity signal match"));
    }
}

void MctpdEndpoint::recover()
{
    try
    {
        connection->async_method_call(
            [weak{weak_from_this()}](const boost::system::error_code& ec
                                     [[maybe_unused]]) {
            if (ec)
            {
                if (auto self = weak.lock())
                {
                    std::cerr << "Failed to recover device at '"
                              << self->objpath.str << "'" << std::endl;
                }
            }
        },
            mctpdBusName, objpath.str, mctpdEndpointControlInterface,
            "Recover");
    }
    catch (const sdbusplus::exception::SdBusError& err)
    {
        std::throw_with_nested(
            MctpException("Failed to schedule endpoint recovery"));
    }
}

void MctpdEndpoint::remove()
{
    try
    {
        connection->async_method_call(
            [self{shared_from_this()}](const boost::system::error_code& ec) {
            if (ec)
            {
                std::cerr << "Failed to remove endpoint @ [" << self->describe()
                          << "]: " << ec.message() << std::endl;
                return;
            }
        },
            mctpdBusName, objpath.str, mctpdEndpointControlInterface, "Remove");
        /* We're driving removal, so no need for notify */
        notifyRemoved = nullptr;
    }
    catch (const sdbusplus::exception::SdBusError& err)
    {
        std::throw_with_nested(
            MctpException("Failed schedule endpoint removal"));
    }
}

void MctpdEndpoint::removed()
{
    if (notifyRemoved)
    {
        notifyRemoved();
    }
}

void MctpdEndpoint::setMtu(
    uint32_t mtu, std::function<void(const std::error_code& ec)>&& completed)
{
    try
    {
        connection->async_method_call(
            [cb{std::move(completed)}](const boost::system::error_code& bsec) {
            cb(static_cast<const std::error_code&>(bsec));
        },
            mctpdBusName, objpath.str, mctpdEndpointControlInterface, "SetMTU",
            mtu);
    }
    catch (const sdbusplus::exception::SdBusError& err)
    {
        completed(std::error_code(err.get_errno(), std::system_category()));
    }
}

std::string MctpdEndpoint::describe()
{
    return std::string("network: ")
        .append(std::to_string(mctp.network))
        .append(", EID: ")
        .append(std::to_string(mctp.eid));
}
