#pragma once

#include <libmctp-smbus.h>
#include <libmctp.h>

#include <NVMeDevice.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <sensor.hpp>

class NVMeMCTPSensor : public Sensor
{
  public:
    NVMeMCTPSensor(sdbusplus::asio::object_server& objectServer,
                   boost::asio::io_service& io,
                   std::shared_ptr<sdbusplus::asio::connection>& conn,
                   const std::string& sensorName,
                   std::vector<thresholds::Threshold>&& _thresholds,
                   const std::string& sensorConfiguration, const int busNumber);
    virtual ~NVMeMCTPSensor();

    NVMeMCTPSensor& operator=(const NVMeMCTPSensor& other) = delete;

    int bus;

  private:
    sdbusplus::asio::object_server& objServer;

    void checkThresholds(void) override;
};

using NVMEMap = boost::container::flat_map<int, std::shared_ptr<NVMeContext>>;

namespace nvmeMCTP
{
// this is add for missing when building.
void init(void);
} // namespace nvmeMCTP

#if ENABLE_MCTP_SMBUS
int mctp_smbus_register_bus(struct mctp_binding_smbus* smbus, struct mctp* mctp,
                            mctp_eid_t eid);
struct mctp_binding_smbus* mctp_smbus_init(void);
int mctp_smbus_open_in_bus(struct mctp_binding_smbus* smbus, int in_bus);
int mctp_smbus_open_out_bus(struct mctp_binding_smbus* smbus, int out_bus);
int mctp_smbus_set_in_fd(struct mctp_binding_smbus* smbus, int fd);
int mctp_smbus_set_out_fd(struct mctp_binding_smbus* smbus, int fd);
int mctp_smbus_get_out_fd(struct mctp_binding_smbus* smbus);
int mctp_smbus_get_in_fd(struct mctp_binding_smbus* smbus);
int mctp_smbus_read(struct mctp_binding_smbus* smbus);
uint32_t crc32c(uint8_t* buf, int len);
int nvmeMessageTransmit(struct mctp& mctp, struct nvme_mi_msg_request& req);
int verifyIntegrity(uint8_t* msg, size_t len);
#else
int mctp_smbus_register_bus(struct mctp_binding_smbus* smbus, struct mctp* mctp,
                            mctp_eid_t eid)
{
    return -ENOTSUP;
}
struct mctp_binding_smbus* mctp_smbus_init(void)
{
    return NULL;
}
int mctp_smbus_open_in_bus(struct mctp_binding_smbus* smbus, int in_bus)
{
    return -ENOTSUP;
}
int mctp_smbus_open_out_bus(struct mctp_binding_smbus* smbus, int out_bus)
{
    return -ENOTSUP;
}
int mctp_smbus_set_in_fd(struct mctp_binding_smbus* smbus, int fd)
{
    return -ENOTSUP;
}
int mctp_smbus_set_out_fd(struct mctp_binding_smbus* smbus, int fd)
{
    return -ENOTSUP;
}
int mctp_smbus_get_out_fd(struct mctp_binding_smbus* smbus)
{
    return -ENOTSUP;
}
int mctp_smbus_get_in_fd(struct mctp_binding_smbus* smbus)
{
    return -ENOTSUP;
}
int mctp_smbus_read(struct mctp_binding_smbus* smbus)
{
    return -ENOTSUP;
}
uint32_t crc32c(uint8_t* buf, int len)
{
    return -ENOTSUP;
}
static inline int nvmeMessageTransmit(struct mctp& mctp,
                                      struct nvme_mi_msg_request& req)
{
    return -ENOTSUP;
}
#endif

NVMEMap& getNVMEMap(void);
