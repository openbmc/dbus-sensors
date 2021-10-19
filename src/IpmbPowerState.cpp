#include <IpmbPowerState.hpp>

#include <iostream>

IpmbPowerMonitor::IpmbPowerMonitor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_service& io, const float pollRate,
    sdbusplus::asio::object_server& objectServer, std::string& name,
    const uint8_t ipmbBusIndex, const uint8_t deviceAddress,
    const std::string& ipmbHostClass,
    const std::vector<std::string>& ipmbStateNames) :
    ipmbPollMs(static_cast<int>(pollRate * 1000)),
    hostName(escapeName(name)), ipmbBusIndex(ipmbBusIndex),
    deviceAddress(deviceAddress), ipmbHostClass(ipmbHostClass),
    ipmbStateNames(ipmbStateNames), objectServer(objectServer),
    dbusConnection(conn), ioTimer(io)
{
    std::string ipmbPowerPathPrefix = "/xyz/openbmc_project/hostcontroller/";
    try
    {
        for (const auto& pathNames : ipmbStateNames)
        {
            sdbusplus::message::object_path dbusPath = ipmbPowerPathPrefix;
            dbusPath /= hostName;
            dbusPath /= escapeName(pathNames);
            std::shared_ptr<sdbusplus::asio::dbus_interface> tmpDbusIface =
                objectServer.add_interface(
                    dbusPath, "xyz.openbmc_project.Chassis.Control.Power");
            tmpDbusIface->register_property("PGood", false);
            if (!tmpDbusIface->initialize())
            {
                std::cerr << "error initializing value interface\n";
                return;
            }
            std::string pathName = escapeName(pathNames);
            std::string ipmbPowerState = pathName.substr(2, pathName.size());

            ipmbPowerInterfaceMap[ipmbPowerState] = tmpDbusIface;
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
        return;
    }
    retryCount++;
}

IpmbStateInfo IpmbPowerMonitor::getHostCtrlProp(std::string propName)
{
    for (const IpmbStateInfo& prop : twinlakeIpmbStateProp)
    {
        if (prop.IpmbState == propName)
        {
            return prop;
        }
    }
    throw std::runtime_error("Invalid config");
}

void IpmbPowerMonitor::setIpmbPowerState(std::vector<uint8_t> ipmbResp)
{
    std::cerr << "in SetIpmbGpioState \n";

    uint8_t ipmbRspLength = 9;

    if (ipmbResp.size() != ipmbRspLength)
    {
        incrementError();
        read();
        return;
    }
    uint8_t stateByteMask = 0;
    uint8_t stateBitMask = 0;

    for (const auto& pathNames : ipmbStateNames)
    {
        std::string ipmbPowerPathPrefix =
            "/xyz/openbmc_project/hostcontroller/";
        sdbusplus::message::object_path dbusPath = ipmbPowerPathPrefix;
        dbusPath /= hostName;
        dbusPath /= escapeName(pathNames);

        std::string pathName = escapeName(pathNames);
        std::string ipmbStateName = pathName.substr(2, pathName.size());
        IpmbStateInfo hostCtrolPropName = getHostCtrlProp(ipmbStateName);

        stateByteMask = static_cast<uint8_t>(hostCtrolPropName.byte);
        stateBitMask = static_cast<uint8_t>(hostCtrolPropName.bit);
        /* First 3 bytes of index is IANA, so it has been ignored
         * we are reading from a register which holds the pgood ipmb
         * state register value is read and stored in ipmbResp
         * variable
         * using the bit and byte mask(stateByteMask and stateBitMask)
         * the power good state is read. */

        bool ipmbState =
            bool(((ipmbResp[stateByteMask] >> stateBitMask) & 1) > 0);
        ipmbPowerInterfaceMap[ipmbStateName]->set_property("PGood", ipmbState);
    }
}

void IpmbPowerMonitor::read()
{
    std::weak_ptr<IpmbPowerMonitor> weakRef = weak_from_this();
    ioTimer.expires_from_now(boost::posix_time::milliseconds(ipmbPollMs));
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
            const int status = std::get<0>(response);
            auto self = weakRef.lock();
            if (!self)
            {
                return;
            }
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
