#pragma once

#include <libmctp-smbus.h>
#include <libmctp.h>

#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/object_server.hpp>
//#include <sensor.hpp>

static int DEBUG = 0;

class NVMeSensor
{
  public:
    NVMeSensor(sdbusplus::asio::object_server& objectServer,
               boost::asio::io_service& io,
               std::shared_ptr<sdbusplus::asio::connection>& conn,
               const std::string& sensorName, int bus, int rootBus);

    NVMeSensor(const NVMeSensor&) = default;
    NVMeSensor(NVMeSensor&&) = default;
    NVMeSensor& operator=(NVMeSensor&& other) = default;
    NVMeSensor& operator=(const NVMeSensor& other) = delete;

    boost::asio::steady_timer& getMctpResponseTimer()
    {
        return mctpResponseTimer;
    }

    void setValue(double value);

    std::shared_ptr<sdbusplus::asio::dbus_interface> sensorInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> association;
    double tempValue = 0.0;

    static void rxMessage(uint8_t eid, void* data, void* msg, size_t len);

    boost::asio::steady_timer mctpResponseTimer;

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

void pollNVMeDevices(boost::asio::io_service& io,
                     boost::asio::steady_timer& scanTimer);

int nvmeMessageTransmit(struct mctp* mctp, uint8_t eid,
                        struct nvme_mi_msg_request* req);

int verifyIntegrity(uint8_t* msg, size_t len);