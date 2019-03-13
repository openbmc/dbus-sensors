#pragma once

#include <libmctp-smbus.h>
#include <libmctp.h>

#include <sensor.hpp>

class NVMeSensor : public Sensor
{
  public:
    NVMeSensor(sdbusplus::asio::object_server& objectServer,
               boost::asio::io_service& io,
               std::shared_ptr<sdbusplus::asio::connection>& conn,
               const std::string& sensorName,
               std::vector<thresholds::Threshold>&& _thresholds,
               const std::string& sensorConfiguration);

    NVMeSensor& operator=(const NVMeSensor& other) = delete;

    boost::asio::deadline_timer* getMctpResponseTimer()
    {
        return mctpResponseTimer.get();
    }

    double value;

  private:
    std::unique_ptr<boost::asio::deadline_timer> mctpResponseTimer;
    void checkThresholds(void) override;
};

using SensorList = std::vector<std::shared_ptr<NVMeSensor>>;

struct NVMeContext
{
    NVMeContext(boost::asio::io_service& io, int bus, int rootBus,
                std::shared_ptr<NVMeSensor>& sensor);

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
    std::shared_ptr<NVMeSensor> sensor;
};

using NVMEList = std::vector<
    std::pair<std::shared_ptr<NVMeSensor>, std::shared_ptr<NVMeContext>>>;

void pollNVMeDevices(boost::asio::io_service& io,
                     boost::asio::deadline_timer& scanTimer,
                     NVMEList& nvmeList);

int nvmeMessageTransmit(struct mctp* mctp, uint8_t eid,
                        struct nvme_mi_msg_request* req);

int verifyIntegrity(uint8_t* msg, size_t len);
