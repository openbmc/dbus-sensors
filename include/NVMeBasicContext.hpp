#pragma once

#include "NVMeContext.hpp"

#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <span>

class NVMeBasicContext : public NVMeContext
{
  public:
    NVMeBasicContext(boost::asio::io_service& io, int rootBus);
    virtual ~NVMeBasicContext() = default;
    virtual void pollNVMeDevices() override;
    virtual void readAndProcessNVMeSensor() override;
    virtual void processResponse(std::span<uint8_t> msg) override;

  private:
    NVMeBasicContext(boost::asio::io_service& io, int rootBus, int cmdOut,
                     int streamIn, int streamOut, int cmdIn);
    boost::asio::io_service& io;
    boost::asio::posix::stream_descriptor reqStream;
    boost::asio::posix::stream_descriptor respStream;
};
