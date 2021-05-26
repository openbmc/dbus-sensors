#pragma once

#include "NVMeContext.hpp"

#include <boost/asio/io_service.hpp>

class NVMeBasicContext : public NVMeContext
{
  public:
    NVMeBasicContext(boost::asio::io_service& io, int rootBus);

    virtual ~NVMeBasicContext();

    virtual void pollNVMeDevices();
    virtual void close();
    virtual void readAndProcessNVMeSensor();
    virtual void processResponse(void* msg, size_t len);

  private:
    boost::asio::io_service& io;
};
