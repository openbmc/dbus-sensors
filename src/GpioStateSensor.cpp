#include <GpioStateSensor.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <iostream>

using IpmbMethodType =
    std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;

static constexpr uint8_t ipmbHostBit = 2;
static constexpr float convertToMs = 1000;
static constexpr uint8_t lun = 0;

static constexpr const char* gpioPathPrefix = "/xyz/openbmc_project/gpio/";
static constexpr const char* gpioInterfaceName =
    "xyz.openbmc_project.Chassis.Control.Power";

IpmbGpio::IpmbGpio(std::shared_ptr<sdbusplus::asio::connection>& conn,
                   boost::asio::io_service& io, const float pollRate,
                   sdbusplus::asio::object_server& objectServer,
                   std::string& name, uint8_t ipmbBusIndex,
                   uint8_t deviceAddress, const std::string& gpioTypeName,
                   const std::string& gpioClass) :
    gpioPollMs(static_cast<int>(pollRate * convertToMs)),
    ipmbBusIndex(ipmbBusIndex), deviceAddress(deviceAddress),
    gpioClass(gpioClass), objectServer(objectServer), dbusConnection(conn),
    ioTimer(io)
{
    name = boost::replace_all_copy(name, " ", "_");

    std::string dbusPath = gpioPathPrefix + gpioTypeName + "/" + name;

    gpioInterface = objectServer.add_interface(dbusPath, gpioInterfaceName);

    gpioInterface->register_property("PGood", bool(false));

    if (!gpioInterface->initialize())
    {
        std::cerr << "error initializing value interface\n";
    }
}

IpmbGpio::~IpmbGpio()
{
    ioTimer.cancel();
    objectServer.remove_interface(gpioInterface);
}

void IpmbGpio::init()
{
    if (gpioClass == "twin_lake_gpio")
    {
        type = IpmbGpioType::gpio;
    }
    else
    {
        std::cerr << "Invalid class " << gpioClass << "\n";
    }
    loadDefaults();
    read();
}

void IpmbGpio::loadDefaults()
{
    if (type == IpmbGpioType::gpio)
    {
        /* IPMB bus index - first 6 bits is device Index and last 2
        bits is IPMB/ME channel. Hence shifting the bus to left
        by 2 bits */
        commandAddress = ipmbBusIndex << ipmbHostBit;
        netfn = oem::twinlake_gpio::netFn;
        command = {deviceAddress};
        commandData = {0x15, 0xa0, 0};
    }
    else
    {
        throw std::runtime_error("Invalid gpio sensor type");
    }
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
                const int& status = std::get<0>(response);
                if (ec || status)
                {
                    std::cerr << " Error reading from IPMB gpio\n";
                    return;
                }

                const std::vector<uint8_t>& data = std::get<5>(response);

                if (data.empty())
                {
                    std::cerr << " IPMB gpio data is empty \n";
                    return;
                }

                /* First 3 bytes of index is IANA, so it has been ignored
                data[registerData] is the GPIO command response
                and first bit denotes the CPU Power Good */
                uint8_t gpioName = data[registerData] & registerPin;
                if (gpioName == 0)
                {
                    gpioInterface->set_property("PGood", bool(false));
                }
                else
                {
                    gpioInterface->set_property("PGood", bool(true));
                }
                read();
            },
            "xyz.openbmc_project.Ipmi.Channel.Ipmb",
            "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
            "sendRequest", commandAddress, netfn, lun, command, commandData);
    });
}
