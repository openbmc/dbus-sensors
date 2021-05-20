#include <unistd.h>

#include <IIOSensor.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <iostream>
#include <istream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

// The pressure units we receive are kilopascal.
// From: https://www.kernel.org/doc/Documentation/ABI/testing/sysfs-bus-iio
// Units after application of scale and offset are kilopascal.
// 1 kilopascal = 10 hectopascal = 1000 Pascals
// The standard unit for used in atmospheric pressure measurements or
// readings is the hectopascal (hPa), in meteorology, for atmospheric
// pressure, the modern equivalent of the traditional millibar.
// However we tend to not put prefixes on dbus APIs, so our pressure
// is in Pascals.
static constexpr unsigned int pressureScaleFactor = 1000;    // a multiplier
static constexpr unsigned int temperatureScaleFactor = 1000; // a divisor

static constexpr double maxReadingPressure = 120000; // Pascals
static constexpr double minReadingPressure = 30000;  // Pascals

static constexpr double maxReadingTemperature = 127;  // DegreesC
static constexpr double minReadingTemperature = -128; // DegreesC

IIOSensor::IIOSensor(const std::string& path, const std::string& objectType,
                     sdbusplus::asio::object_server& objectServer,
                     std::shared_ptr<sdbusplus::asio::connection>& conn,
                     boost::asio::io_service& io, const std::string& sensorName,
                     std::vector<thresholds::Threshold>&& thresholdsIn,
                     const float pollRate,
                     const std::string& sensorConfiguration,
                     const PowerState powerState,
                     const std::string& sensorType) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholdsIn), sensorConfiguration, objectType, false,
           sensorType.compare("pressure") ? maxReadingTemperature
                                          : maxReadingPressure,
           sensorType.compare("pressure") ? minReadingTemperature
                                          : minReadingPressure,
           conn, powerState),
    std::enable_shared_from_this<IIOSensor>(), objServer(objectServer),
    inputDev(io, open(path.c_str(), O_RDONLY)), waitTimer(io), path(path),
    sensorPollMs(static_cast<unsigned int>(pollRate * 1000))
{
    const char* units;

    if (sensorType.compare("temperature") == 0)
    {
        units = "DegreesC";
    }
    else if (sensorType.compare("pressure") == 0)
    {
        units = "Pascals";
    }
    // This should NEVER happen.
    else
    {
        units = "Unknown";
    }

    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/" + sensorType + "/" + name,
        "xyz.openbmc_project.Sensor.Value");

    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/" + sensorType + "/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/" + sensorType + "/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Critical");
    }
    association = objectServer.add_interface("/xyz/openbmc_project/sensors/" +
                                                 sensorType + "/" + name,
                                             association::interface);
    setInitialProperties(conn, units);
}

IIOSensor::~IIOSensor()
{
    // close the input dev to cancel async operations
    inputDev.close();
    waitTimer.cancel();
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(thresholdInterfaceCritical);
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void IIOSensor::setupRead(void)
{
    std::weak_ptr<IIOSensor> weakRef = weak_from_this();

    boost::asio::async_read_until(inputDev, readBuf, '\n',
                                  [weakRef](const boost::system::error_code& ec,
                                            std::size_t /*bytes_transfered*/) {
                                      std::shared_ptr<IIOSensor> self =
                                          weakRef.lock();
                                      if (self)
                                      {
                                          self->handleResponse(ec);
                                      }
                                  });
}

void IIOSensor::handleResponse(const boost::system::error_code& err)
{
    if ((err == boost::system::errc::bad_file_descriptor) ||
        (err == boost::asio::error::misc_errors::not_found))
    {
        std::cerr << "IIO sensor " << name << " removed " << path << "\n";
        return; // we're being destroyed
    }
    std::istream responseStream(&readBuf);
    if (!err)
    {
        std::string response;
        std::getline(responseStream, response);
        try
        {
            rawValue = std::stod(response);
            double nvalue;
            std::string myPressure = "in_pressure";
            std::string myTemperature = "in_temp";
            if (path.find(myPressure) != std::string::npos)
            {
                nvalue = rawValue * pressureScaleFactor;
            }
            else if (path.find(myTemperature) != std::string::npos)
            {
                nvalue = rawValue / temperatureScaleFactor;
            }
            // This should NEVER happen.
            else
            {
                nvalue = rawValue * 1.0;
            }
            updateValue(nvalue);
        }
        catch (const std::invalid_argument&)
        {
            incrementError();
        }
    }
    else
    {
        incrementError();
    }

    responseStream.clear();
    inputDev.close();
    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0)
    {
        std::cerr << "IIO sensor " << name << " not valid " << path << "\n";
        return; // we're no longer valid
    }
    inputDev.assign(fd);
    waitTimer.expires_from_now(boost::posix_time::milliseconds(sensorPollMs));
    std::weak_ptr<IIOSensor> weakRef = weak_from_this();
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        std::shared_ptr<IIOSensor> self = weakRef.lock();
        if (ec == boost::asio::error::operation_aborted)
        {
            if (self)
            {
                std::cerr << "IIO sensor " << self->name << " read cancelled "
                          << self->path << "\n";
            }
            else
            {
                std::cerr << "IIO sensor read cancelled, no self\n";
            }
            return; // we're being canceled
        }
        if (self)
        {
            self->setupRead();
        }
    });
}

void IIOSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}
