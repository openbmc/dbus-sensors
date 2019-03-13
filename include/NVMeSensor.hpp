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
#include <mctp/libmctp-smbus.h>
#include <mctp/libmctp.h>
#include <mctp/nvme-mi.h>
}

static constexpr double maxReading = 127;
static constexpr double minReading = -128;

class NVMeSensor;

using MctpDeviceData = std::shared_ptr<struct nvmeContext>;
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
                              struct nvmeContext& nvmeDevice,
                              boost::asio::io_service& io,
                              boost::asio::ip::tcp::socket& nvmeSlaveSocket,
                              boost::asio::deadline_timer& tcpResponseTimer);
int nvmeMessageTransmit(struct mctp* mctp, uint8_t eid,
                        struct nvme_mi_msg_request* req);

static int verifyIntegrity(uint8_t* msg, size_t len);

struct nvmeContext : std::enable_shared_from_this<nvmeContext>
{
    nvmeContext(boost::asio::io_service& io, int _bus,
                struct mctp_binding_smbus* _smbus) :
        bus(_bus)
    {
        eid = 0;
        smbus = mctp_smbus_init();
        memcpy(smbus, _smbus, sizeof(*smbus));
        mctp = mctp_init();

        int r = mctp_smbus_open_out_bus(smbus, bus);
        if (r < 0)
        {
            return;
        }

        mctp_smbus_register_bus(smbus, mctp, eid);

        mctp_set_rx_all(mctp, rxMessage, NULL);
    }

    ~nvmeContext()
    {
    }

    nvmeContext(const nvmeContext&);
    static void rxMessage(uint8_t _eid, void* data, void* msg, size_t len);

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
               const std::string& sensorConfiguration, size_t bus) :
        Sensor(boost::replace_all_copy(sensorName, " ", "_"), path,
               std::move(_thresholds), sensorConfiguration,
               "xyz.openbmc_project.Configuration.NVMe", maxReading,
               minReading),
        objServer(objectServer), inputDev(io, open(path.c_str(), O_RDONLY)),
        waitTimer(io), errCount(0), thresholdTimer(io, this), bus(bus),
        nvmeMasterSocket(io)
    {
        sensorInterface = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/temperature/" + name,
            "xyz.openbmc_project.Sensor.Value");
        if (thresholds::hasWarningInterface(thresholds))
        {
            thresholdInterfaceWarning = objectServer.add_interface(
                "/xyz/openbmc_project/sensors/temperature/" + name,
                "xyz.openbmc_project.Sensor.Threshold.Warning");
        }
        if (thresholds::hasCriticalInterface(thresholds))
        {
            thresholdInterfaceCritical = objectServer.add_interface(
                "/xyz/openbmc_project/sensors/temperature/" + name,
                "xyz.openbmc_project.Sensor.Threshold.Critical");
        }

        setInitialProperties(conn);
        setupRead();

        // setup match
        setupPowerMatch(conn);
    }

    void updateNVMeReading(const double& value)
    {
        updateValue(value);
    }

    virtual ~NVMeSensor()
    {
        // close the input dev to cancel async operations
        inputDev.close();
        waitTimer.cancel();
        objServer.remove_interface(sensorInterface);
    }

  private:
    sdbusplus::asio::object_server& objServer;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    boost::asio::streambuf readBuf;
    int errCount;
    thresholds::ThresholdTimer thresholdTimer;
    int bus; // Bus this drive is on
    boost::asio::posix::stream_descriptor nvmeMasterSocket;
    std::shared_ptr<struct nvmeContext> readingContext;

    void setupRead()
    {
    }

    void checkThresholds(void) override
    {
    }
};
