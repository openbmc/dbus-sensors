#include <GpioStateSensor.hpp>

#include <iostream>

IpmbPowerMonitor::IpmbPowerMonitor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_service& io, const float pollRate,
    sdbusplus::asio::object_server& objectServer, std::string& name,
    const uint8_t ipmbBusIndex, const uint8_t deviceAddress,
    const std::string& ipmbHostClass,
    const std::vector<std::string>& ipmbStateNames) :
    gpioPollMs(static_cast<int>(pollRate * 1000)),
    hostName(escapeName(name)), ipmbBusIndex(ipmbBusIndex),
    deviceAddress(deviceAddress), ipmbHostClass(ipmbHostClass),
    ipmbStateNames(ipmbStateNames), objectServer(objectServer),
    dbusConnection(conn), ioTimer(io)
{
    const char* ipmbPowerPathPrefix = "/xyz/openbmc_project/hostcontroller/";
    try
    {
        for (auto& pathNames : ipmbStateNames)
        {
            std::string pathName = escapeName(pathNames);
            std::string dbusPath =
                ipmbPowerPathPrefix + hostName + "/" + pathName;
            std::string ipmbPowerState = pathName.substr(2, pathName.size());

            std::shared_ptr<sdbusplus::asio::dbus_interface> tmpDbusIface =
                objectServer.add_interface(
                    dbusPath, "xyz.openbmc_project.Chassis.Control.Power");
            tmpDbusIface->register_property("PGood", false);
            if (!tmpDbusIface->initialize())
            {
                std::cerr << "error initializing value interface\n";
                return;
            }
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

void IpmbPowerMonitor::SetIpmbPowerState(std::vector<uint8_t> ipmbResp)
{
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
        const char* ipmbPowerPathPrefix =
            "/xyz/openbmc_project/hostcontroller/";
        std::string pathName = escapeName(pathNames);
        std::string dbusPath = ipmbPowerPathPrefix + hostName + "/" + pathName;
        std::string gpioStateName = pathName.substr(2, pathName.size());

        IpmbStateInfo hostCtrolPropName = getHostCtrlProp(gpioStateName);
        stateByteMask = static_cast<uint8_t>(hostCtrolPropName.byte);
        stateBitMask = static_cast<uint8_t>(hostCtrolPropName.bit);
        /* First 3 bytes of index is IANA, so it has been ignored
         * we are reading from a register which holds the pgood gpio
         * state register value is read and stored in ipmbgpiodata
         * variable
         * using the bit mask(stateBitMask) the pgood state is read. */

        bool gpioState =
            bool(((ipmbResp[stateByteMask] >> stateBitMask) & 1) > 0);
        ipmbPowerInterfaceMap[gpioStateName]->set_property("PGood", gpioState);
    }
}

void IpmbPowerMonitor::read()
{
    ioTimer.expires_from_now(boost::posix_time::milliseconds(gpioPollMs));
    std::weak_ptr<IpmbPowerMonitor> weakRef = weak_from_this();
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
            if (ec || status)
            {
                self->incrementError();
                self->read();
                return;
            }
            const std::vector<uint8_t> ipmbResp = std::get<5>(response);

            self->SetIpmbPowerState(ipmbResp);
            self->read();
            },
            "xyz.openbmc_project.Ipmi.Channel.Ipmb",
            "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
            "sendRequest", self->commandAddress, self->netfn, lun,
            self->deviceAddress, self->commandData);
    });
}
