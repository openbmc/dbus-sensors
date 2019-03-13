#pragma once

#include <crc32c.h>
#include <libmctp-smbus.h>
#include <libmctp.h>
#include <nvme-mi.h>

#include <Thresholds.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/basic_stream_socket.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/optional/optional.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

static constexpr double maxReading = 127;
static constexpr double minReading = -128;
static int DEBUG = 0;

struct NVMeContext
{
    NVMeContext(boost::asio::io_service& io, int _bus, int _rootBus);
    NVMeContext(const NVMeContext&);
    static void rxMessage(uint8_t _eid, void* data, void* msg, size_t len);
    ~NVMeContext(){};

    int bus; // Bus this drive is on
    // MCTP specific
    std::unique_ptr<struct mctp_binding_smbus,
                    std::function<void(mctp_binding_smbus*)>>
        smbus;
    struct mctp* mctp;
    mctp_eid_t eid;
    int rootBus; // Root bus for this drive
    std::shared_ptr<boost::asio::ip::tcp::socket> nvmeSlaveSocket;
};

class NVMeSensor : public Sensor
{
  public:
    NVMeSensor(const std::string& path,
               sdbusplus::asio::object_server& objectServer,
               std::shared_ptr<sdbusplus::asio::connection>& conn,
               boost::asio::io_service& io, const std::string& sensorName,
               std::vector<thresholds::Threshold>&& _thresholds,
               const std::string& sensorConfiguration, size_t bus);

    void updateNVMeReading(const double& value);
    virtual ~NVMeSensor();

  private:
    thresholds::ThresholdTimer thresholdTimer;
    int bus; // Bus this drive is on

    void checkThresholds(void) override;
};

using MctpDeviceData = std::shared_ptr<struct NVMeContext>;
using SensorInfo = std::unique_ptr<NVMeSensor>;

extern std::vector<std::pair<SensorInfo, MctpDeviceData>> nvmeDeviceList;
extern size_t lastQueriedDeviceIndex;

void readAndProcessNVMeSensor(NVMeSensor& sensorInfo,
                              struct NVMeContext& nvmeDevice,
                              boost::asio::io_service& io,
                              boost::asio::deadline_timer& mctpResponseTimer);
int nvmeMessageTransmit(struct mctp* mctp, uint8_t eid,
                        struct nvme_mi_msg_request* req);

int verifyIntegrity(uint8_t* msg, size_t len);