#pragma once

#include "NVMeContext.hpp"

#include <boost/asio/io_service.hpp>

class NVMeBasicContext : public NVMeContext
{
  public:
    NVMeBasicContext(boost::asio::io_service& io, int rootBus);
    virtual ~NVMeBasicContext() = default;
    virtual void pollNVMeDevices() override;
    virtual void readAndProcessNVMeSensor() override;
    virtual void processResponse(void* msg, size_t len) override;

  private:
    boost::asio::io_service& io;
};
