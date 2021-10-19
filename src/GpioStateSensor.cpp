#include <GpioStateSensor.hpp>

#include <iostream>

static constexpr float convertToMs = 1000;
static constexpr uint8_t ipmbGpioMax = 9;

static constexpr const char* gpioPathPrefix = "/xyz/openbmc_project/ipmbgpio/";
static constexpr const char* gpioInterfaceName =
    "xyz.openbmc_project.Chassis.Control.Power";
inline gpioMapType gpioMapName;
const char* gpioDbusProperty = "PGood";

IpmbGpioStateMonitor::IpmbGpioStateMonitor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_service& io, const float pollRate,
    sdbusplus::asio::object_server& objectServer, std::string& name,
    const uint8_t ipmbBusIndex, const uint8_t deviceAddress,
    const std::string& gpioClass,
    const std::vector<std::string>& ipmbGpioStates) :
    gpioPollMs(static_cast<int>(pollRate * convertToMs)),
    gpioName(escapeName(name)), ipmbBusIndex(ipmbBusIndex),
    deviceAddress(deviceAddress), gpioClass(gpioClass),
    ipmbGpioStates(ipmbGpioStates), objectServer(objectServer),
    dbusConnection(conn), ioTimer(io)
{
    try
    {
        for (auto& pathNames : ipmbGpioStates)
        {
            std::string pathName = escapeName(pathNames);
            std::string gpioState = pathName.substr(2, pathName.size());
            std::string dbusPath = gpioPathPrefix + gpioName + "/" + pathName;
            gpioInterface =
                objectServer.add_interface(dbusPath, gpioInterfaceName);
            gpioInterface->register_property(gpioDbusProperty, bool(false));
            if (!gpioInterface->initialize())
            {
                std::cerr << "error initializing value interface\n";
                return;
            }
            gpioInterfaceMap.insert({gpioState, gpioInterface});
        }
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << gpioClass << "\n";
        return;
    }
    read();
}

IpmbGpioStateMonitor::~IpmbGpioStateMonitor()
{
    ioTimer.cancel();
    objectServer.remove_interface(gpioInterface);
}

void IpmbGpioStateMonitor::incrementError()
{
    if (retryCount >= 3)
    {
        retryCount = 0;
        return;
    }
    retryCount++;
}

gpioMapType IpmbGpioStateMonitor::getGpioMap()
{
    std::string gpioNames = gpioName.substr(2, gpioName.size());
    if (gpioNames == "TwinlakeipmbGpiostate")
    {
        return twinlakeIpmbgpioMap;
    }
    throw std::runtime_error("Invalid config");
}

void IpmbGpioStateMonitor::read()
{
    ioTimer.expires_from_now(boost::posix_time::milliseconds(gpioPollMs));
    ioTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        dbusConnection->async_method_call(
            [this](boost::system::error_code ec,
                   const IpmbMethodType& response) {
                const int status = std::get<0>(response);
                if (ec || status)
                {
                    incrementError();
                    read();
                    return;
                }

                const std::vector<uint8_t> ipmbGpioData = std::get<5>(response);

                /* First 3 bytes of index is IANA, so it has been ignored
                 * we are reading from a register which holds the pgood gpio
                 * state register value is read and stored in ipmbgpiodata
                 * variable
                 * using the bit mask(gpioBitMask) the pgood state is read. */

                if (ipmbGpioData.size() != ipmbGpioMax)
                {
                    incrementError();
                    read();
                    return;
                }

                gpioMapName = getGpioMap();
                for (auto& pathNames : ipmbGpioStates)
                {
                    std::string pathName = escapeName(pathNames);
                    std::string dbusPath =
                        gpioPathPrefix + gpioName + "/" + pathName;
                    std::string gpioStateName =
                        pathName.substr(2, pathName.size());
                    gpioByteMask =
                        static_cast<uint8_t>(gpioMapName[gpioStateName].byte);
                    gpioBitMask =
                        static_cast<uint8_t>(gpioMapName[gpioStateName].bit);

                    bool gpioState = bool(
                        ((ipmbGpioData[gpioByteMask] >> gpioBitMask) & 1) > 0);
                    gpioInterfaceMap[gpioStateName]->set_property(
                        gpioDbusProperty, gpioState);
                }
                read();
            },
            "xyz.openbmc_project.Ipmi.Channel.Ipmb",
            "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
            "sendRequest", commandAddress, netfn, lun, deviceAddress,
            commandData);
    });
}
