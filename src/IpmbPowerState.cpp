#include <IpmbPowerState.hpp>

#include <iostream>

IpmbPowerMonitor::IpmbPowerMonitor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_service& io, const float pollRate,
    sdbusplus::asio::object_server& objectServer, std::string& name,
    const uint8_t ipmbBusIndex, const uint8_t deviceAddress,
    const std::string& ipmbHostClass) :
    ipmbPollMs(static_cast<int>(pollRate * 1000)),
    hostName(escapeName(name)), ipmbBusIndex(ipmbBusIndex),
    deviceAddress(deviceAddress), ipmbHostClass(ipmbHostClass),
    objectServer(objectServer), dbusConnection(conn), ioTimer(io)
{
    const char* ipmbPowerPathPrefix = "/xyz/openbmc_project/hostcontroller/";
    try
    {
        for (const auto& pathNames : twinlakeIpmbStateProp)
        {
            sdbusplus::message::object_path dbusPath =
                static_cast<std::string>(ipmbPowerPathPrefix);
            dbusPath /= hostName;
            dbusPath /= pathNames.ipmbState;

            std::shared_ptr<sdbusplus::asio::dbus_interface> tmpDbusIface =
                objectServer.add_interface(dbusPath, pathNames.intfName);
            tmpDbusIface->register_property(pathNames.ipmbState, false);

            if (!tmpDbusIface->initialize())
            {
                std::cerr << "error initializing value interface\n";
                return;
            }
            ipmbPowerInterfaceMap[pathNames.ipmbState] = tmpDbusIface;
        }
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << ipmbHostClass << "\n";
        throw e;
    }
}

IpmbPowerMonitor::~IpmbPowerMonitor()
{
    ioTimer.cancel();
    for (const auto& iface : ipmbPowerInterfaceMap)
    {
        objectServer.remove_interface(iface.second);
    }
}

void IpmbPowerMonitor::incrementError()
{
    if (retryCount >= 3)
    {
        retryCount = 0;
        throw std::runtime_error("Error Ipmb response failed");
    }
    retryCount++;
}

IpmbStateInfo IpmbPowerMonitor::getHostCtrlProp(const std::string& propName)
{
    for (IpmbStateInfo& prop : twinlakeIpmbStateProp)
    {
        if (prop.ipmbState == propName)
        {
            return prop;
        }
    }
    throw std::runtime_error("Invalid config");
}

void IpmbPowerMonitor::setIpmbPowerState(std::vector<uint8_t> ipmbResp)
{
    if (ipmbResp.size() != ipmbRspLength)
    {
        incrementError();
        read();
        return;
    }
    retryCount = 0;
    uint8_t stateByteMask = 0;
    uint8_t stateBitMask = 0;

    for (const auto& pathNames : twinlakeIpmbStateProp)
    {
        IpmbStateInfo hostCtrolPropName = getHostCtrlProp(pathNames.ipmbState);
        stateByteMask = static_cast<uint8_t>(hostCtrolPropName.byte);
        stateBitMask = static_cast<uint8_t>(hostCtrolPropName.bit);
        /* First 3 bytes of index is IANA, so it has been ignored
         * we are reading from a register which holds the pgood gpio
         * state register value is read and stored in ipmbgpiodata
         * variable
         * using the bit mask(stateBitMask) the pgood state is read. */
        if (hostCtrolPropName.byte > ByteMask::BYTE_8)
        {
            std::cerr << "return byte is less than requied\n";
            return;
        }
        bool ipmbState = ((ipmbResp[stateByteMask]) & stateBitMask) != 0;
        ipmbPowerInterfaceMap[hostCtrolPropName.ipmbState]->set_property(
            hostCtrolPropName.ipmbState, ipmbState);
    }
}

void IpmbPowerMonitor::read()
{
    std::weak_ptr<IpmbPowerMonitor> weakRef = weak_from_this();
    ioTimer.expires_from_now(std::chrono::milliseconds(ipmbPollMs));
    ioTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        auto self = weakRef.lock();
        if (!self)
        {
            return;
        }
        self->dbusConnection->async_method_call(
            [weakRef](boost::system::error_code ec,
                      const IpmbMethodType& response) {
            auto self = weakRef.lock();
            if (!self)
            {
                return;
            }
            const int status = std::get<0>(response);
            if (ec || (status != 0))
            {
                self->incrementError();
                self->read();
                return;
            }
            const std::vector<uint8_t> ipmbResp = std::get<5>(response);

            self->setIpmbPowerState(ipmbResp);
            self->read();
            },
            "xyz.openbmc_project.Ipmi.Channel.Ipmb",
            "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
            "sendRequest", self->commandAddress, self->netfn, lun,
            self->deviceAddress, self->commandData);
    });
}
