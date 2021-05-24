#pragma once

#include "NVMeContext.hpp"

#include <boost/asio/ip/tcp.hpp>

class NVMeMCTPContext : public NVMeContext
{
  public:
    NVMeMCTPContext(boost::asio::io_service& io, int rootBus);

    virtual ~NVMeMCTPContext();

    virtual void pollNVMeDevices() override;
    virtual void close() override;
    virtual void readAndProcessNVMeSensor() override;
    virtual void processResponse(void* msg, size_t len) override;

  private:
    boost::asio::ip::tcp::socket nvmeSlaveSocket;
    boost::asio::deadline_timer mctpResponseTimer;

    void readResponse();
};

namespace nvmeMCTP
{
void init(void);
}
