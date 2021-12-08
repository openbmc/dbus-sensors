#pragma once

#include <boost/asio/io_service.hpp>
#include <sensor.hpp>

class NVMeSensor : public Sensor
{
  public:
    static constexpr const char* CONFIG_TYPE =
        "xyz.openbmc_project.Configuration.NVME1000";

    NVMeSensor(sdbusplus::asio::object_server& objectServer,
               boost::asio::io_service& io,
               std::shared_ptr<sdbusplus::asio::connection>& conn,
               const std::string& sensorName,
               std::vector<thresholds::Threshold>&& thresholds,
               const std::string& sensorConfiguration, const int busNumber);
    virtual ~NVMeSensor();

    NVMeSensor& operator=(const NVMeSensor& other) = delete;

    int bus;

  private:
    sdbusplus::asio::object_server& objServer;

    void checkThresholds(void) override;
};
