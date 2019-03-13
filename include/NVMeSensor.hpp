#pragma once

#include <Thresholds.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/basic_stream_socket.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/optional/optional.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

extern "C" {
#include <mctp/crc32c.h>
#include <mctp/libmctp-smbus.h>
#include <mctp/libmctp.h>
#include <mctp/nvme-mi.h>
}

static constexpr double maxReading = 127;
static constexpr double minReading = -128;

struct NVMeContext
{
    NVMeContext(boost::asio::io_service& io, int _bus, int _rootBus);
    NVMeContext(const NVMeContext&);
    static void rxMessage(uint8_t _eid, void* data, void* msg, size_t len);
    ~NVMeContext();

    int bus; // Bus this drive is on
    // MCTP specific
    struct mctp_binding_smbus* smbus;
    struct mctp* mctp;
    mctp_eid_t eid;
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
    sdbusplus::asio::object_server& objServer;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    boost::asio::streambuf readBuf;
    int errCount;
    thresholds::ThresholdTimer thresholdTimer;
    int bus; // Bus this drive is on
    boost::asio::posix::stream_descriptor nvmeMasterSocket;
    std::shared_ptr<struct NVMeContext> readingContext;

    void checkThresholds(void) override;
};

using MctpDeviceData = std::shared_ptr<struct NVMeContext>;
using SensorInfo = std::unique_ptr<NVMeSensor>;
using NvmeDeviceList = boost::container::flat_map<
    int, std::pair<SensorInfo, MctpDeviceData>>; // bus, <sensorinfo, mctp>

extern boost::container::flat_map<int, NvmeDeviceList>
    deviceRootBusMap; // root bus, map of busses containing drives

extern boost::container::flat_map<
    int, std::pair<struct mctp_binding_smbus*,
                   std::shared_ptr<boost::asio::ip::tcp::socket>>>
    rootBusFileDescriptors;                   // bus , fd
extern std::pair<int, int> lastQueriedDevice; // device was on this rootbus, bus

void readAndProcessNVMeSensor(NVMeSensor& sensorInfo,
                              struct NVMeContext& nvmeDevice,
                              boost::asio::io_service& io,
                              boost::asio::ip::tcp::socket& nvmeSlaveSocket,
                              boost::asio::deadline_timer& tcpResponseTimer);
int nvmeMessageTransmit(struct mctp* mctp, uint8_t eid,
                        struct nvme_mi_msg_request* req);

static int verifyIntegrity(uint8_t* msg, size_t len);