#include <IpmbPowerState.hpp>

#include <iostream>

IpmbPowerMonitor::IpmbPowerMonitor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const float pollRate,
    sdbusplus::asio::object_server& objectServer, std::string& name,
    const uint8_t ipmbBusIndex, const uint8_t deviceAddress,
    const std::string& ipmbHostClass) :
    ipmbPollMs(static_cast<int>(pollRate * 1000)),
    hostName(escapeName(name)), ipmbBusIndex(ipmbBusIndex),
    deviceAddress(deviceAddress), ipmbHostClass(ipmbHostClass),
    objectServer(objectServer), dbusConnection(conn), ioTimer(io)
{
    std::string ipmbPowerPathPrefix =
        "/xyz/openbmc_project/Chassis/Control/Power";
    try
    {
        ipmbPowerPathPrefix += std::to_string(ipmbBusIndex + 1);
        sdbusplus::message::object_path dbusPath = ipmbPowerPathPrefix;
        std::shared_ptr<sdbusplus::asio::dbus_interface> tmpDbusIface =
            objectServer.add_interface(
                dbusPath, "xyz.openbmc_project.Chassis.Control.Power");
        tmpDbusIface->register_property("PGood", false);
        if (!tmpDbusIface->initialize())
        {
            throw std::runtime_error("error initializing value interface\n");
        }

        ipmbPowerInterfaceMap[prop.IpmbState] = tmpDbusIface;
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
    if (retryCount >= retryTimes)
    {
        retryCount = 0;
        std::cerr << "Error Ipmb response failed \n";
        return;
    }
    retryCount++;
}

void IpmbPowerMonitor::setIpmbPowerState(std::vector<uint8_t> ipmbResp)
{
    if (ipmbResp.size() != ipmbRspLength)
    {
        incrementError();
        return;
    }
    retryCount = 0;
    uint8_t stateByteMask = 0;
    uint8_t stateBitMask = 0;

    stateByteMask = static_cast<uint8_t>(prop.byte);
    stateBitMask = static_cast<uint8_t>(prop.bit);
    if (prop.byte > ByteMask::BYTE_8)
    {
        return;
    }
    /* First 3 bytes of index is IANA, so it has been ignored
     * we are reading from a register which holds the pgood gpio
     * state register value is read and stored in ipmbgpiodata
     * variable
     * using the bit mask(stateBitMask) the pgood state is read. */
    bool ipmbState = ((ipmbResp[stateByteMask]) & stateBitMask) != 0;
    ipmbPowerInterfaceMap[prop.IpmbState]->set_property("PGood", ipmbState);
}

void IpmbPowerMonitor::read()
{
    std::weak_ptr<IpmbPowerMonitor> weakRef = weak_from_this();
    ioTimer.expires_after(std::chrono::milliseconds(ipmbPollMs));
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
