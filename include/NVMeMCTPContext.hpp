#pragma once

#include "NVMeContext.hpp"

#include <boost/asio/ip/tcp.hpp>
#include <span>

class NVMeMCTPContext : public NVMeContext
{
  public:
    NVMeMCTPContext(boost::asio::io_service& io, int rootBus);

    virtual ~NVMeMCTPContext();

    virtual void pollNVMeDevices() override;
    virtual void close() override;
    virtual void readAndProcessNVMeSensor() override;
    virtual void processResponse(std::span<uint8_t> msg) override;

  private:
    boost::asio::ip::tcp::socket nvmeSlaveSocket;
    boost::asio::deadline_timer mctpResponseTimer;

    void readResponse();
};

namespace nvmeMCTP
{
void init(void);
}
