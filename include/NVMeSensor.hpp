#pragma once

#include <NVMeDevice.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <sensor.hpp>

class NVMeSensor : public Sensor
{
  public:
    NVMeSensor(sdbusplus::asio::object_server& objectServer,
               boost::asio::io_service& io,
               std::shared_ptr<sdbusplus::asio::connection>& conn,
               const std::string& sensorName,
               std::vector<thresholds::Threshold>&& _thresholds,
               const std::string& sensorConfiguration, const int busNumber);
    virtual ~NVMeSensor();

    NVMeSensor& operator=(const NVMeSensor& other) = delete;

    int bus;

  private:
    sdbusplus::asio::object_server& objServer;

    void checkThresholds(void) override;
};

struct NVMeContext : std::enable_shared_from_this<NVMeContext>
{
    NVMeContext(boost::asio::io_service& io, int rootBus);

    virtual ~NVMeContext();

    virtual void pollNVMeDevices();
    virtual void close();

    int rootBus; // Root bus for this drive

    boost::asio::deadline_timer scanTimer;

    boost::asio::deadline_timer mctpResponseTimer;
    boost::asio::ip::tcp::socket nvmeSlaveSocket;

    std::list<std::shared_ptr<NVMeSensor>> sensors; // used as a poll queue
};
struct NVMeSMBusContext : NVMeContext
{
    int busfd;

    // link NVMeContext

    virtual ~NVMeSMBusContext();
};

using NVMEMap = boost::container::flat_map<int, std::shared_ptr<NVMeContext>>;

int verifyIntegrity(uint8_t* msg, size_t len);

namespace nvmeSMBus
{
void init(void);
}

NVMEMap& getNVMEMap(void);
