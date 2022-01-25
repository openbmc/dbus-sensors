#pragma once

#include "NVMeContext.hpp"

#include <boost/asio/ip/tcp.hpp>

class NVMeMCTPContext : public NVMeContext
{
  public:
    NVMeMCTPContext(boost::asio::io_service& io, int rootBus);

    ~NVMeMCTPContext() override;

    void pollNVMeDevices() override;
    void close() override;
    void readAndProcessNVMeSensor() override;
    void processResponse(void* msg, size_t len) override;

  private:
    boost::asio::ip::tcp::socket nvmeSlaveSocket;
    boost::asio::deadline_timer mctpResponseTimer;

    void readResponse();
};

namespace nvme_mctp
{
void init(void);
}
