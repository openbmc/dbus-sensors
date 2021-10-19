#include <GpioStateSensor.hpp>

#include <iostream>

static inline uint8_t ipmbHostBit = 2;
static inline float convertToMs = 1000;

static inline const char* gpioPathPrefix = "/xyz/openbmc_project/gpio/";
static inline const char* gpioInterfaceName =
    "xyz.openbmc_project.Chassis.Control.Power";
inline std::string gpioMapName;
const char* gpioDbusProperty = "PGood";

IpmbGpioStateMonitor::IpmbGpioStateMonitor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_service& io, const float pollRate,
    sdbusplus::asio::object_server& objectServer, std::string& name,
    const uint8_t ipmbBusIndex, const uint8_t deviceAddress,
    const std::string& gpioTypeName, const std::string& gpioClass) :
    gpioPollMs(static_cast<int>(pollRate * convertToMs)),
    gpioName(escapeName(name)), ipmbBusIndex(ipmbBusIndex),
    deviceAddress(deviceAddress), gpioClass(gpioClass),
    objectServer(objectServer), dbusConnection(conn), ioTimer(io)
{
    try
    {
        gpioMapName = gpioName.substr(2, sizeof(gpioName));
        if (init())
        {
            std::string dbusPath =
                gpioPathPrefix + gpioTypeName + "/" + gpioName;

            gpioInterface =
                objectServer.add_interface(dbusPath, gpioInterfaceName);

            gpioInterface->register_property(gpioDbusProperty, bool(false));

            if (!gpioInterface->initialize())
            {
                std::cerr << "error initializing value interface\n";
                return;
            }
        }
        else
        {
            throw std::runtime_error("Failed to initialize");
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

bool IpmbGpioStateMonitor::init()
{
    if (gpioClass == "twin_lake_gpio")
    {
        /* IPMB bus index - first 6 bits is device Index and last 2
        bits is IPMB/ME channel. Hence shifting the bus to left
        by 2 bits */
        commandAddress = ipmbBusIndex << ipmbHostBit;
        netfn = oem::twinlake_gpio::netFn;
        gpioByteMask = twinlake_ipmbgpio_map[gpioMapName].byteMask;
        gpioBitMask = twinlake_ipmbgpio_map[gpioMapName].bitMask;
    }
    else
    {
        std::cerr << "Invalid class " << gpioClass << " \n";
        return false;
    }
    return true;
}

void IpmbGpioStateMonitor::incrementError()
{
    if (retryCount >= 3)
    {
        setProperty(false);
        retryCount = 0;
        return;
    }
    retryCount++;
}

void IpmbGpioStateMonitor::setProperty(bool value)
{
    gpioInterface->set_property(gpioDbusProperty, value);
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
                 * using the bit mask(pgoodgpiobit) the pgood state is read. */

                if (gpioByteMask >= ipmbGpioData.size())
                {
                    incrementError();
                    read();
                    return;
                }

                gpioState =
                    bool((ipmbGpioData[gpioByteMask] & gpioBitMask) > 0);

                setProperty(gpioState);

                read();
            },
            "xyz.openbmc_project.Ipmi.Channel.Ipmb",
            "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
            "sendRequest", commandAddress, netfn, lun, deviceAddress,
            commandData);
    });
}
