#pragma once

#include <crc32c.h>
#include <libmctp-smbus.h>
#include <libmctp.h>

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
static constexpr double minReading = -60;
static int DEBUG = 0;

struct NVMeContext
{
    NVMeContext(boost::asio::io_service& io, int bus, int rootBus);
    NVMeContext(const NVMeContext&) = default;
    NVMeContext(NVMeContext&&) = default;

    static void rxMessage(uint8_t eid, void* data, void* msg, size_t len);
    int bus; // Bus this drive is on
    // MCTP specific
    std::unique_ptr<struct mctp_binding_smbus,
                    std::function<void(mctp_binding_smbus*)>>
        smbus;
    struct mctp* mctp;
    mctp_eid_t eid;
    int rootBus; // Root bus for this drive
    std::shared_ptr<boost::asio::ip::tcp::socket> nvmeSlaveSocket;
    int sindex; // passed to callback to update the sensor info
};

class NVMeSensor : public Sensor
{
  public:
    NVMeSensor(sdbusplus::asio::object_server& objectServer,
               boost::asio::io_service& io,
               std::shared_ptr<sdbusplus::asio::connection>& conn,
               const std::string& sensorName,
               std::vector<thresholds::Threshold>&& _thresholds,
               const std::string& sensorConfiguration);

    NVMeSensor(const NVMeSensor&) = default;
    NVMeSensor(NVMeSensor&&) = default;
    NVMeSensor& operator=(const NVMeSensor& other) = delete;
    NVMeSensor& operator=(NVMeSensor&& other)
    {
        if (this != &other)
        {
            mctpResponseTimer = std::move(other.mctpResponseTimer);
            other.mctpResponseTimer.release();
        }
        return *this;
    }

    boost::asio::deadline_timer* getMctpResponseTimer()
    {
        return mctpResponseTimer.get();
    }

    double value;

  private:
    std::unique_ptr<boost::asio::deadline_timer> mctpResponseTimer;
    void checkThresholds(void) override;
};

extern std::vector<std::pair<NVMeSensor, std::shared_ptr<struct NVMeContext>>>
    nvmeDeviceList;

void pollNVMeDevices(boost::asio::io_service& io,
                     boost::asio::deadline_timer& scanTimer);

int nvmeMessageTransmit(struct mctp* mctp, uint8_t eid,
                        struct nvme_mi_msg_request* req);

int verifyIntegrity(uint8_t* msg, size_t len);