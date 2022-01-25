#include <GpioStateSensor.hpp>

#include <iostream>

using IpmbMethodType =
    std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;

static constexpr uint8_t ipmbHostBit = 2;
static constexpr float convertToMs = 1000;
static constexpr uint8_t lun = 0;
static uint8_t count = 0;

static constexpr const char* gpioPathPrefix = "/xyz/openbmc_project/gpio/";
static constexpr const char* gpioInterfaceName =
    "xyz.openbmc_project.Chassis.Control.Power";
const char* pgoodDbusProperty = "PGood";

IpmbGpio::IpmbGpio(std::shared_ptr<sdbusplus::asio::connection>& conn,
                   boost::asio::io_service& io, const float pollRate,
                   sdbusplus::asio::object_server& objectServer,
                   std::string& name, const uint8_t ipmbBusIndex,
                   const uint8_t deviceAddress, const std::string& gpioTypeName,
                   const std::string& gpioClass, const uint8_t pgoodGpioByte,
                   const uint8_t pgoodGpioBit) :
    gpioPollMs(static_cast<int>(pollRate * convertToMs)),
    gpioName(escapeName(name)), ipmbBusIndex(ipmbBusIndex),
    deviceAddress(deviceAddress), gpioClass(gpioClass),
    pgoodGpioByte(pgoodGpioByte), pgoodGpioBit(pgoodGpioBit),
    objectServer(objectServer), dbusConnection(conn), ioTimer(io)
{
    if (init())
    {
        std::string dbusPath = gpioPathPrefix + gpioTypeName + "/" + gpioName;

        gpioInterface = objectServer.add_interface(dbusPath, gpioInterfaceName);

        gpioInterface->register_property(pgoodDbusProperty, bool(false));

        if (!gpioInterface->initialize())
        {
            std::cerr << "error initializing value interface\n";
            return;
        }
        read();
    }
}

IpmbGpio::~IpmbGpio()
{
    ioTimer.cancel();
    objectServer.remove_interface(gpioInterface);
}

bool IpmbGpio::init()
{
    if (gpioClass == "twin_lake_gpio")
    {
        type = IpmbGpioType::ipmbGpio;
    }
    else
    {
        std::cerr << "Invalid class " << gpioClass << "\n";
        return false;
    }
    return loadDefaults();
}

bool IpmbGpio::loadDefaults()
{
    if (type == IpmbGpioType::ipmbGpio)
    {
        /* IPMB bus index - first 6 bits is device Index and last 2
        bits is IPMB/ME channel. Hence shifting the bus to left
        by 2 bits */
        commandAddress = ipmbBusIndex << ipmbHostBit;
        netfn = oem::twinlake_gpio::netFn;
        commandData = {0x15, 0xa0, 0};
    }
    else
    {
        std::cerr << "Invalid gpio sensor type \n";
        return false;
    }
    return true;
}

void IpmbGpio::read()
{
    ioTimer.expires_from_now(boost::posix_time::milliseconds(gpioPollMs));
    ioTimer.async_wait([&](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        dbusConnection->async_method_call(
            [&](boost::system::error_code ec, const IpmbMethodType& response) {
                const int status = std::get<0>(response);
                if (ec || status)
                {
                    if (count >= 3)
                    {
                        std::cerr
                            << " Error reading from IPMB gpio retrying...\n";
                        gpioInterface->set_property(pgoodDbusProperty,
                                                    bool(false));
                        count = 0;
                    }
                    count++;
                    read();
                    return;
                }

                const std::vector<uint8_t> ipmbGpioData = std::get<5>(response);

                /* First 3 bytes of index is IANA, so it has been ignored
                 * we are reading from a register which holds the pgood gpio
                 * state register value is read and stored in ipmbgpiodata
                 * variable
                 * using the bit mask(pgoodgpiobit) the pgood state is read. */

                if (pgoodGpioByte >= ipmbGpioData.size())
                {
                    if (count >= 3)
                    {
                        std::cerr
                            << " IPMB gpio data is not populated retrying...\n";
                        gpioInterface->set_property(pgoodDbusProperty,
                                                    bool(false));
                        count = 0;
                    }
                    count++;
                    read();
                    return;
                }

                bool powerGoodValue =
                    ipmbGpioData[pgoodGpioByte] & pgoodGpioBit;
                gpioInterface->set_property(pgoodDbusProperty, powerGoodValue);

                read();
            },
            "xyz.openbmc_project.Ipmi.Channel.Ipmb",
            "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
            "sendRequest", commandAddress, netfn, lun, deviceAddress,
            commandData);
    });
}
