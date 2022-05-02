#pragma once

#include "NVMeContext.hpp"

#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>

class NVMeBasicContext : public NVMeContext
{
  public:
    NVMeBasicContext(boost::asio::io_service& io, int rootBus);
    ~NVMeBasicContext() override = default;
    void pollNVMeDevices() override;
    void readAndProcessNVMeSensor() override;
    void processResponse(std::shared_ptr<NVMeSensor>& sensor, void* msg,
                         size_t len) override;

  private:
    NVMeBasicContext(boost::asio::io_service& io, int rootBus, int cmdOut,
                     int streamIn, int streamOut, int cmdIn);
    boost::asio::io_service& io;
    boost::asio::posix::stream_descriptor reqStream;
    boost::asio::posix::stream_descriptor respStream;

    enum
    {
        NVME_MI_BASIC_SFLGS_DRIVE_NOT_READY = 0x40,
        NVME_MI_BASIC_SFLGS_DRIVE_FUNCTIONAL = 0x20,
    };
};
