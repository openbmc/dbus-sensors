#pragma once

#include "NVMeContext.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>

#include <thread>

class NVMeBasicContext : public NVMeContext
{
  public:
    NVMeBasicContext(boost::asio::io_context& io, int rootBus);
    ~NVMeBasicContext() override = default;
    void pollNVMeDevices() override;
    void readAndProcessNVMeSensor() override;
    void processResponse(std::shared_ptr<NVMeSensor>& sensor, void* msg,
                         size_t len) override;

  private:
    NVMeBasicContext(boost::asio::io_context& io, int rootBus, int cmdOut,
                     int streamIn, int streamOut, int cmdIn);
    boost::asio::io_context& io;

    // The IO thread must be destructed after the stream descriptors, so
    // initialise it first. http://eel.is/c++draft/class.base.init#note-6
    //
    // Providing a stop-source to the thread execution function isn't
    // particularly useful as it will spend most of its time blocked in a system
    // call - ioctl() for the actual device communication, or read() and write()
    // on the pipes associated with reqStream and respStream. Rather than trying
    // to force a stop, rely on read()/write() failures from closed pipes to
    // coerce it to exit and thus allow completion of the join().
    std::jthread thread;

    // Destruction of the stream descriptors has the effect of issuing cancel(),
    // destroying the closure of the callback where we might be carrying
    // weak_ptrs to `this`.
    // https://www.boost.org/doc/libs/1_79_0/doc/html/boost_asio/reference/posix__basic_descriptor/_basic_descriptor.html
    boost::asio::posix::stream_descriptor reqStream;
    boost::asio::posix::stream_descriptor respStream;

    enum
    {
        NVME_MI_BASIC_SFLGS_DRIVE_NOT_READY = 0x40,
        NVME_MI_BASIC_SFLGS_DRIVE_FUNCTIONAL = 0x20,
    };
};
